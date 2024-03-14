// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawSubsystem;

public class ShootCommand extends Command {

  private final ClawSubsystem clawSubsystem;

  private final Timer timer = new Timer();
  private final double delay;

  public ShootCommand(ClawSubsystem clawSubsystem, double delay) {
    this.clawSubsystem = clawSubsystem;
    this.delay = delay;

    addRequirements(clawSubsystem);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
  }

  @Override
  public void execute() {
    if(timer.get() > delay) {
      clawSubsystem.setIntake(.80);
    }
    clawSubsystem.setOuttake(.95);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    clawSubsystem.disableMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer.get() > 5;
  }
}
