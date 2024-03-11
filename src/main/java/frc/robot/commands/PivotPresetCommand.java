// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawPivotSubsystem;
import frc.robot.subsystems.ClawSubsystem;

public class PivotPresetCommand extends Command {
  private final ClawPivotSubsystem clawPivotSubsystem;

  private double goal;


  public PivotPresetCommand(ClawPivotSubsystem clawPivotSubsystem, double goal) {
    this.clawPivotSubsystem = clawPivotSubsystem;

    this.goal = goal;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if ((clawPivotSubsystem.getPosition() != (goal += 2.0)) && (clawPivotSubsystem.getPosition() < goal)) {
      clawPivotSubsystem.setSpeed(-.4);
    } else if((clawPivotSubsystem.getPosition() != (goal -= 2.0)) && (clawPivotSubsystem.getPosition() > goal)) {
      clawPivotSubsystem.setSpeed(.4);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    clawPivotSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
