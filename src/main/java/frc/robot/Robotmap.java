// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Robotmap {
  public static class SwerveDrive {
    public static int LEFT_FRONT_DRIVE_SPEED_MOTOR_PIN = 25;
    public static int LEFT_BACK_DRIVE_SPEED_MOTOR_PIN = 27;
    public static int RIGHT_FRONT_DRIVE_SPEED_MOTOR_PIN = 28;
    public static int RIGHT_BACK_DRIVE_SPEED_MOTOR_PIN = 26;
   
    public static int LEFT_FRONT_DRIVE_DIRECTION_MOTOR_PIN = 8;
    public static int LEFT_BACK_DRIVE_DIRECTION_MOTOR_PIN = 10;
    public static int RIGHT_FRONT_DRIVE_DIRECTION_MOTOR_PIN = 12;
    public static int RIGHT_BACK_DRIVE_DIRECTION_MOTOR_PIN = 7;


    public static int LEFT_FRONT_DRIVE_DIRECTION_ENCODER_PIN = 17;

    public static int LEFT_BACK_DRIVE_DIRECTION_ENCODER_PIN = 15;
  

    public static int RIGHT_FRONT_DRIVE_DIRECTION_ENCODER_PIN = 16;
  

    public static int RIGHT_BACK_DRIVE_DIRECTION_ENCODER_PIN = 18;

    public static double DRIVE_DIRECTION_P =  0.02;
    public static double DRIVE_DIRECTION_I = 0.001;
    public static double DRIVE_DIRECTION_D = 0;




  }
}
