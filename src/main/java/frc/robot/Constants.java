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
public final class Constants {
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kControlControllerPort = 1;
    public static final double controllerDeadzone = 0.1;
  }

  public static final int falconEncoderResolution = 2048;
  public static final double swerveWheelDiameter = 0.09677;
  public static final double swerveDriveGearRatio = 6.75;
  public static final double swerveTurnGearRatio = 150.0 / 7.0;
  public static final double moduleMaxAngularVelocity = Math.PI;
  public static final double moduleMaxAngularAcceleration = 2 * Math.PI;
  public static final double maxModuleVelocity = 6.5; 
  /*this variable changes robot speek 
  max speed is 16. speed is in m/s. do not make robot go too fast*/

  public static final double maxModuleAcceleration = 6.5;
  //changes speed
  public static final double maxTranslationalDriveSpeed = 6.5;
  public static final double maxRotationalDriveSpeed = -Math.PI;
  public static final double controllerDeadzone = 0.1;

  //encoder offsets, stil around a 1 degree error
  public static final double backLeftEncoderOffset = 1.07378655151;
  public static final double backRightEncoderOffset = -0.90044672248;
  public static final double frontLeftEncoderOffset = -0.15953400194;
  public static final double frontRightEncoderOffset = 0.09664078963;
  public static final double armGearRatio = 1280.0;
  public static final double clawGearRatio = 396.19;

  public static final double maxArmSpeed = 0.6;
  public static final double maxPivotSpeed = 0.6;

}
