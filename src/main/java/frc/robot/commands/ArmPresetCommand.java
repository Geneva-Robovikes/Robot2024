// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawPivotSubsystem;
import frc.robot.subsystems.ClawSubsystem;

public class ArmPresetCommand extends Command {
  private double goal;
  private double Kp;
  private ArmSubsystem armSubsystem;

  /** Creates a new ArmPresetCommand. */
  public ArmPresetCommand(ArmSubsystem armSubsystem, double goal) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.goal = goal;
    this.armSubsystem = armSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentPosition = armSubsystem.getArmMotor1Position();
    double error = goal - currentPosition;
    double command = error * Kp;
    armSubsystem.setArmSpeed(command);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (armSubsystem.getArmMotor1Position() == goal);
  }
}
