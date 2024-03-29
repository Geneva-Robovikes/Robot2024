// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class TeleopCommand extends Command {
  private final DriveSubsystem driveSubsystem;
  private final CommandXboxController driveController;

  private final double maxSpeedXY = Constants.maxTranslationalDriveSpeed;
  private final double maxSpeedTheta = Constants.maxRotationalDriveSpeed;
  boolean isFieldCentric = true;

  /**
   * Drives the robot based off the controller inputs.
   * @param driveSubsystem the drive subsystem
   * @param driveController the controller to take input from
   */
  public TeleopCommand(DriveSubsystem driveSubsystem, CommandXboxController driveController) {
    this.driveSubsystem = driveSubsystem;
    this.driveController = driveController;
    addRequirements(driveSubsystem);
  }
  
  @Override
  public void execute() {
    // Squares inputs so fine movements are easier to make
    double x1 = Math.signum(driveController.getLeftX()) * Math.pow(driveController.getLeftX(), 2);
    double y1 = Math.signum(-driveController.getLeftY()) * Math.pow(driveController.getLeftY(), 2);
    double x2 = Math.signum(driveController.getRightX()) * Math.pow(driveController.getRightX(), 2);
    double rightTrigger = driveController.getRightTriggerAxis();
    boolean resetButton = driveController.y().getAsBoolean();

    // If the right controller is pressed, drive switches to robot centric
    if(rightTrigger > 0.5) {
      isFieldCentric = true;
    } else {
      isFieldCentric = true;
    }

    if(resetButton) {
      driveSubsystem.resetGyro();
    }

    //controlls to reset gyro and swerve
    if(driveController.rightTrigger().getAsBoolean()) {
      driveSubsystem.resetGyro();
    }else if(driveController.y().getAsBoolean()){
      driveSubsystem.swerveAlignment();
    }

    // Applies a deadzone to the controller inpts
    x1 = MathUtil.applyDeadband(x1, OperatorConstants.controllerDeadzone);
    y1 = MathUtil.applyDeadband(y1, OperatorConstants.controllerDeadzone);
    x2 = MathUtil.applyDeadband(x2, OperatorConstants.controllerDeadzone);

    // Switches inputs to speeds in meters per second
    double vX = y1 * maxSpeedXY;  
    double vY = x1 * maxSpeedXY;
    double vTheta = x2 * maxSpeedTheta;

    // Sets the drive speeds
    driveSubsystem.setModuleStatesFromSpeeds(vX, vY, vTheta, true);
  }

  @Override
  public void end(boolean interrupted) {
    driveSubsystem.setModuleStatesFromSpeeds(0, 0, 0, true);
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}