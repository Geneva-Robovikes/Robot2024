// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.ArmSubsystem;

public class ArmCommand extends Command {
  //there was a suppress warning thingy on falcon command so idk what that is

  private final ArmSubsystem armSubsystem;
  /** Creates a new ArmCommand. */
  public ArmCommand(ArmSubsystem subsystem) {
    armSubsystem = subsystem;

    addRequirements(subsystem);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("Arm speed set to 30%");

    armSubsystem.setspeed(0.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Arm speed set to 0%");

    armSubsystem.setspeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
