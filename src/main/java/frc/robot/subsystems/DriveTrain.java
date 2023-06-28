// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.I2C.Port;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.Robotmap;

public class DriveTrain extends SubsystemBase {
  private static TalonSRX LEFT_FRONT_DRIVE_SPEED_MOTOR;
  private static TalonSRX LEFT_BACK_DRIVE_SPEED_MOTOR;
  private static TalonSRX RIGHT_FRONT_DRIVE_SPEED_MOTOR;
  private static TalonSRX RIGHT_BACK_DRIVE_SPEED_MOTOR;

  private static TalonSRX LEFT_FRONT_DRIVE_DIRECTION_MOTOR;
  private static TalonSRX LEFT_BACK_DRIVE_DIRECTION_MOTOR;
  private static TalonSRX RIGHT_FRONT_DRIVE_DIRECTION_MOTOR;
  private static TalonSRX RIGHT_BACK_DRIVE_DIRECTION_MOTOR;

  // Encoders
  public static Encoder LEFT_FRONT_DRIVE_DISTANCE_ENCODER;
  public static Encoder LEFT_BACK_DRIVE_DISTANCE_ENCODER;
  public static Encoder RIGHT_FRONT_DRIVE_DISTANCE_ENCODER;
  public static Encoder RIGHT_BACK_DRIVE_DISTANCE_ENCODER;
  public static PIDController DRIVE_DISTANCE_ENCODERS;

  public static Encoder LEFT_FRONT_DRIVE_DIRECTION_ENCODER;
  public static Encoder LEFT_BACK_DRIVE_DIRECTION_ENCODER;
  public static Encoder RIGHT_FRONT_DRIVE_DIRECTION_ENCODER;
  public static Encoder RIGHT_BACK_DRIVE_DIRECTION_ENCODER;

  // Direction encoder wrapper that scales to degrees
  public static PIDController LEFT_FRONT_DRIVE_DIRECTION_CONTROLLER;
  public static PIDController LEFT_BACK_DRIVE_DIRECTION_CONTROLLER;
  public static PIDController RIGHT_FRONT_DRIVE_DIRECTION_CONTROLLER;
  public static PIDController RIGHT_BACK_DRIVE_DIRECTION_CONTROLLER;

  // Gyro
  public static AHRS DRIVE_GYRO;

  SwerveDriveWheel LEFT_FRONT_DRIVE_WHEEL;
  SwerveDriveWheel LEFT_BACK_DRIVE_WHEEL;
  SwerveDriveWheel RIGHT_FRONT_DRIVE_WHEEL;
  SwerveDriveWheel RIGHT_BACK_DRIVE_WHEEL;
  /** Creates a new SwerveDrive. */
  public SwerveDriveCoordinator SWERVE_DRIVE_COORDINATOR;

  public DriveTrain() {
     // Motors
     LEFT_FRONT_DRIVE_SPEED_MOTOR = new TalonSRX(Robotmap.SwerveDrive.LEFT_FRONT_DRIVE_SPEED_MOTOR_PIN);
     LEFT_BACK_DRIVE_SPEED_MOTOR = new TalonSRX(Robotmap.SwerveDrive.LEFT_BACK_DRIVE_SPEED_MOTOR_PIN);
     RIGHT_FRONT_DRIVE_SPEED_MOTOR = new TalonSRX(Robotmap.SwerveDrive.RIGHT_FRONT_DRIVE_SPEED_MOTOR_PIN);
     RIGHT_BACK_DRIVE_SPEED_MOTOR = new TalonSRX(Robotmap.SwerveDrive.RIGHT_BACK_DRIVE_SPEED_MOTOR_PIN);

     LEFT_FRONT_DRIVE_DIRECTION_MOTOR = new TalonSRX(Robotmap.SwerveDrive.LEFT_FRONT_DRIVE_DIRECTION_MOTOR_PIN);
     LEFT_BACK_DRIVE_DIRECTION_MOTOR = new TalonSRX(Robotmap.SwerveDrive.LEFT_BACK_DRIVE_DIRECTION_MOTOR_PIN);
     RIGHT_FRONT_DRIVE_DIRECTION_MOTOR = new TalonSRX(Robotmap.SwerveDrive.RIGHT_FRONT_DRIVE_DIRECTION_MOTOR_PIN);
     RIGHT_BACK_DRIVE_DIRECTION_MOTOR = new TalonSRX(Robotmap.SwerveDrive.RIGHT_BACK_DRIVE_DIRECTION_MOTOR_PIN);

     // Encoders
     LEFT_FRONT_DRIVE_DISTANCE_ENCODER = new Encoder(Robotmap.SwerveDrive.LEFT_FRONT_DRIVE_DISTANCE_ENCODER_PIN_A, Robotmap.SwerveDrive.LEFT_FRONT_DRIVE_DISTANCE_ENCODER_PIN_B);
     LEFT_BACK_DRIVE_DISTANCE_ENCODER = new Encoder(Robotmap.SwerveDrive.LEFT_BACK_DRIVE_DISTANCE_ENCODER_PIN_A, Robotmap.SwerveDrive.LEFT_BACK_DRIVE_DISTANCE_ENCODER_PIN_B);
     RIGHT_FRONT_DRIVE_DISTANCE_ENCODER = new Encoder(Robotmap.SwerveDrive.RIGHT_FRONT_DRIVE_DISTANCE_ENCODER_PIN_A, Robotmap.SwerveDrive.RIGHT_FRONT_DRIVE_DISTANCE_ENCODER_PIN_B);
     RIGHT_BACK_DRIVE_DISTANCE_ENCODER = new Encoder(Robotmap.SwerveDrive.RIGHT_BACK_DRIVE_DISTANCE_ENCODER_PIN_A, Robotmap.SwerveDrive.RIGHT_BACK_DRIVE_DISTANCE_ENCODER_PIN_B);
     //DRIVE_ENCODERS = new MedianPIDSource(LEFT_FRONT_DRIVE_DISTANCE_ENCODER, LEFT_BACK_DRIVE_DISTANCE_ENCODER, RIGHT_FRONT_DRIVE_DISTANCE_ENCODER, RIGHT_BACK_DRIVE_DISTANCE_ENCODER);

     LEFT_FRONT_DRIVE_DIRECTION_ENCODER = new Encoder(Robotmap.SwerveDrive.LEFT_FRONT_DRIVE_DIRECTION_ENCODER_PIN_A, Robotmap.SwerveDrive.LEFT_FRONT_DRIVE_DIRECTION_ENCODER_PIN_B);
     LEFT_BACK_DRIVE_DIRECTION_ENCODER = new Encoder(Robotmap.SwerveDrive.LEFT_BACK_DRIVE_DIRECTION_ENCODER_PIN_A, Robotmap.SwerveDrive.LEFT_BACK_DRIVE_DIRECTION_ENCODER_PIN_B);
     RIGHT_FRONT_DRIVE_DIRECTION_ENCODER = new Encoder(Robotmap.SwerveDrive.RIGHT_FRONT_DRIVE_DIRECTION_ENCODER_PIN_A, Robotmap.SwerveDrive.RIGHT_FRONT_DRIVE_DIRECTION_ENCODER_PIN_B);
     RIGHT_BACK_DRIVE_DIRECTION_ENCODER = new Encoder(Robotmap.SwerveDrive.RIGHT_BACK_DRIVE_DIRECTION_ENCODER_PIN_A, Robotmap.SwerveDrive.RIGHT_BACK_DRIVE_DIRECTION_ENCODER_PIN_B);

     // Direction encoder wrapper that scales to degrees
     LEFT_FRONT_DRIVE_DIRECTION_CONTROLLER = new PIDController(Robotmap.SwerveDrive.DRIVE_DIRECTION_P,Robotmap.SwerveDrive.DRIVE_DIRECTION_I,Robotmap.SwerveDrive.DRIVE_DIRECTION_D);
     LEFT_BACK_DRIVE_DIRECTION_CONTROLLER = new PIDController(Robotmap.SwerveDrive.DRIVE_DIRECTION_P,Robotmap.SwerveDrive.DRIVE_DIRECTION_I,Robotmap.SwerveDrive.DRIVE_DIRECTION_D);
     RIGHT_FRONT_DRIVE_DIRECTION_CONTROLLER = new PIDController(Robotmap.SwerveDrive.DRIVE_DIRECTION_P,Robotmap.SwerveDrive.DRIVE_DIRECTION_I,Robotmap.SwerveDrive.DRIVE_DIRECTION_D);
     RIGHT_BACK_DRIVE_DIRECTION_CONTROLLER = new PIDController(Robotmap.SwerveDrive.DRIVE_DIRECTION_P,Robotmap.SwerveDrive.DRIVE_DIRECTION_I,Robotmap.SwerveDrive.DRIVE_DIRECTION_D);

     // Gyro
     DRIVE_GYRO = new AHRS(Port.kMXP);

      // SwerveDriveWheels
      double wheelP = 0.02;
      double wheelI = 0.001;
      double wheelD = 0.0;
      LEFT_FRONT_DRIVE_WHEEL = new SwerveDriveWheel(wheelP, wheelI, wheelD, LEFT_FRONT_DRIVE_DIRECTION_ENCODER, LEFT_FRONT_DRIVE_DIRECTION_MOTOR, LEFT_FRONT_DRIVE_SPEED_MOTOR);
      LEFT_BACK_DRIVE_WHEEL = new SwerveDriveWheel(wheelP, wheelI, wheelD, LEFT_BACK_DRIVE_DIRECTION_ENCODER, LEFT_BACK_DRIVE_DIRECTION_MOTOR, LEFT_BACK_DRIVE_SPEED_MOTOR);
      RIGHT_FRONT_DRIVE_WHEEL = new SwerveDriveWheel(wheelP, wheelI, wheelD, RIGHT_FRONT_DRIVE_DIRECTION_ENCODER, RIGHT_FRONT_DRIVE_DIRECTION_MOTOR, RIGHT_FRONT_DRIVE_SPEED_MOTOR);
      RIGHT_BACK_DRIVE_WHEEL = new SwerveDriveWheel(wheelP, wheelI, wheelD, RIGHT_BACK_DRIVE_DIRECTION_ENCODER, RIGHT_BACK_DRIVE_DIRECTION_MOTOR, RIGHT_BACK_DRIVE_SPEED_MOTOR); 
      // SwerveDriveCoordinator
      SWERVE_DRIVE_COORDINATOR = new SwerveDriveCoordinator(LEFT_FRONT_DRIVE_WHEEL, LEFT_BACK_DRIVE_WHEEL, RIGHT_FRONT_DRIVE_WHEEL, RIGHT_BACK_DRIVE_WHEEL);


  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
