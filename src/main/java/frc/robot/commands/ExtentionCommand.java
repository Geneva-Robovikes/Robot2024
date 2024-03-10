// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;

public class ExtentionCommand extends Command {
  private final ArmSubsystem armSubsystem;
  
  private final int direction;

  public ExtentionCommand(ArmSubsystem armSubsystem, int direction) {
    this.armSubsystem = armSubsystem;

    this.direction = direction;

    addRequirements(armSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (direction == 0) {
      armSubsystem.setExtensionSpeed(0.11);
    } else if (direction == 1) {
      armSubsystem.setExtensionSpeed(-0.11);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    armSubsystem.setExtensionSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
