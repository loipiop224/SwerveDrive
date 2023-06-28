// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenixpro.StatusSignalValue;
import com.ctre.phoenixpro.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;

/** Add your docs here. */
public class SwerveDriveWheel {

    public PIDController directionController;
    public CANcoder directionCANcoder;
    public WPI_TalonFX directionMotor;
    public WPI_TalonFX speedMotor;
   // public PIDOutput directionMotor;
   // public PIDSource directionSensor;

   public SwerveDriveWheel(double P, double I, double D,CANcoder dirCANcoder, WPI_TalonFX dirMotor,WPI_TalonFX speedMotor)
   {
        this.directionMotor = dirMotor;
        this.directionCANcoder =  dirCANcoder;
        this.speedMotor = speedMotor;
       this.directionController = new PIDController(P, I, D);
   }
   public Double getCurrentAngle(){
        return directionCANcoder.getPosition().getValue();
   }

   public void setDirection(double setpoint)
   {
       directionController.reset();
       double currentAngle = getCurrentAngle();
       directionController.setSetpoint(currentAngle + closestAngle(currentAngle, setpoint));
    }

    private static double closestAngle(double a, double b)
    {
            // get direction
            double dir = (b%360.0) - (a%360.0);

            // convert from -360 to 360 to -180 to 180
            if (Math.abs(dir) > 180.0)
            {
                    dir = -(Math.signum(dir) * 360.0) + dir;
            }
            return dir;
    }

    public void setSpeed(double speed)
    {
        speedMotor.set( speed);
    }



}
