// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbCommand extends Command {

  private final ClimbSubsystem climbSubsystem;
  double voltage;

  public ClimbCommand(ClimbSubsystem climbSubsystem, double voltage) {
    this.climbSubsystem = climbSubsystem;
    this.voltage = voltage;

    addRequirements(climbSubsystem);
  }

  @Override
  public void initialize() {
    climbSubsystem.setClimbVolts(voltage);
  }

  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climbSubsystem.setClimbVolts(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
