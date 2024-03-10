// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawPivotSubsystem;
import frc.robot.subsystems.ClawSubsystem;

public class PresetCommand extends Command {
  private final ClawPivotSubsystem clawPivotSubsystem;
  private final ClawSubsystem clawSubsystem;
  private final ArmSubsystem armSubsystem;

  private final int mode;

  public PresetCommand(ClawSubsystem clawSubsystem, ArmSubsystem armSubsystem, ClawPivotSubsystem clawPivotSubsystem, int mode) {
    this.clawPivotSubsystem = clawPivotSubsystem;
    this.clawSubsystem = clawSubsystem;
    this.armSubsystem = armSubsystem;

    this.mode = mode;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    switch(mode) {
      case 0:
        System.out.println(clawPivotSubsystem.getPosition() + " " + armSubsystem.getArmMotor1Position() + " " + armSubsystem.getArmMotor2Position());
        break;
      case 1:
        break;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    clawPivotSubsystem.setSpeed(0);
    armSubsystem.setArmSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
