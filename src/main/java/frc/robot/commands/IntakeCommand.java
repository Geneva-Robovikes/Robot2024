// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawSubsystem;

public class IntakeCommand extends Command {
  private final ClawSubsystem clawSubsystem;
  private final Timer timer = new Timer();
  private final double currentLimit;
  private final double delay;
  private final double speed;

  public IntakeCommand(ClawSubsystem subsystem, double speed, double currentLimit, double delay) {
    clawSubsystem = subsystem;
    this.speed = speed;
    this.delay = delay;
    this.currentLimit = currentLimit;

    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    clawSubsystem.setIntake(0.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    clawSubsystem.disableMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(timer.get() > delay) {
      return clawSubsystem.getClawMotorCurrent() > currentLimit;
    }else {
      return false;
    }
  }
}
