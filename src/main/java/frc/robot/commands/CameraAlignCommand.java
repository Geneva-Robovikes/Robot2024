// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.ClawPivotSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class CameraAlignCommand extends Command {
  private final CameraSubsystem cameraSubsystem;
  private final DriveSubsystem driveSubsystem;
  private final ClawPivotSubsystem clawPivotSubsystem;
  PIDController controller;
  double targetYaw;
  double targetPitch;
  double targetDistance;
  double angularVelocity;
  double Kp = .01;

  /** Creates a new CameraAlignCommand. */
  public CameraAlignCommand(CameraSubsystem cameraSubsystem, DriveSubsystem driveSubsystem, ClawPivotSubsystem clawPivotSubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.cameraSubsystem = cameraSubsystem;
    this.driveSubsystem = driveSubsystem;
    this.clawPivotSubsystem = clawPivotSubsystem;

    controller = new PIDController(.1, 0, 0);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(cameraSubsystem.hasTargets()){
      targetYaw = cameraSubsystem.getTarget1Yaw();
      targetPitch = cameraSubsystem.getTarget1Pitch();

      angularVelocity = controller.calculate(targetYaw,0);

      driveSubsystem.setModuleStatesFromSpeeds(0, 0, angularVelocity, true);
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.setModuleStatesFromSpeeds(0,0,0,true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return controller.atSetpoint() ;
  }
}
