// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.Arrays;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class SwerveDrive extends CommandBase {
  private double xMove;
  private double yMove;
  private double rotation;

  //The 2 weights should add up to 1
  private double translationWeight = 0.75;
  private double rotationWeight = 0.25;

  private XboxController controller;
  private DriveTrain driveTrain;

  private double[][] translationVectors;
  private double[][] rotationVectors;
  private double[][] combinedVectors;

  /** Creates a new SwerveDrive. */
  public SwerveDrive(XboxController controller, DriveTrain driveTrain) {
    this.controller = controller;
    this.driveTrain = driveTrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveTrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    translationVectors = new double[4][2];
    rotationVectors = new double[4][2];
    combinedVectors = new double[4][2];
  }
  
  //works
  private void calculateTranslation(){
    double magnitude = 0;
    double direction = 0;
    if(!(xMove == 0 && yMove == 0)){//Run if there are translation inputs
      magnitude = Math.sqrt(Math.pow(xMove, 2) + Math.pow(yMove, 2));
      direction = Math.acos(xMove*(1/magnitude)) * 180/Math.PI;
      if(yMove < 0){
        direction = 360-direction;
      }
    }

    for(double[] vector : translationVectors) {
      vector[0] = direction;
      vector[1] = magnitude;
    }
  }
  
  //works
  private void calculateRotation(){
    double[] moduleAngles = {45, 135, 225, 315};
    double gyroAngle = 45;
    for(int i = 0; i < 4; i++){
      double magnitude = Math.abs(rotation);
      double direction = moduleAngles[i] - gyroAngle;
      if(rotation < 0){ //left
        direction += 90;
      }else{ //right
        direction -= 90;
      }
      direction %= 360;
      if(direction < 0){
        direction += 360;
      }

      rotationVectors[i][0] = direction;
      rotationVectors[i][1] = magnitude;
    }
  }

  //doesn't work
  private void combineAndScaleVectors(){
    for(int i = 0; i < 4; i++){
      double x = Math.cos(translationVectors[i][0]) * translationVectors[i][1] * translationWeight;
      double y = Math.sin(translationVectors[i][0]) * translationVectors[i][1] * translationWeight;
      x += Math.cos(rotationVectors[i][0]) * rotationVectors[i][1] * rotationWeight;
      y += Math.sin(rotationVectors[i][0]) * rotationVectors[i][1] * rotationWeight;

      double magnitude = 0;
      double direction = 0;
      if(!(xMove == 0 && yMove == 0)){
        magnitude = Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2));
        direction = Math.acos(x*(1/magnitude)) * 180/Math.PI;
        if(yMove < 0){
          direction = 360-direction;
        }
      }
      // magnitude = Math.min(magnitude, 1);

      combinedVectors[i][0] = direction;
      combinedVectors[i][1] = magnitude;
    }
  }
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    xMove = MathUtil.applyDeadband(controller.getLeftX(), 0.05);
    yMove = MathUtil.applyDeadband(-controller.getLeftY(), 0.05);
    rotation = MathUtil.applyDeadband(controller.getRightX(), 0.05);

    calculateTranslation();
    calculateRotation();
    combineAndScaleVectors();
    
    // System.out.println(rotation);
    // System.out.println(Arrays.deepToString(combinedVectors));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
