// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.Clawpivotsubsystem;

public class clawpivotdowncommand extends Command {
  private Clawpivotsubsystem subsystem;

  /** Creates a new clawpivotcommand. */
  public clawpivotdowncommand(Clawpivotsubsystem subsystem) {
    Clawpivotsubsystem sub;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Clawpivotsubsystem.setspeed(-0.3);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Clawpivotsubsystem.setspeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return subsystem.getlimit();
  }
}
