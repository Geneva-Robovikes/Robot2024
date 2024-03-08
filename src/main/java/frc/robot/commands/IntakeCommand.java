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
    clawSubsystem = subsystem; // the subsystem
    this.speed = speed; // the speed
    this.delay = delay; // the delay
    this.currentLimit = currentLimit; // the current limit

    addRequirements(subsystem); // adds the requirements 
  }

  @Override
  public void initialize() { // initialiazes
    timer.reset(); // resets the timer
    timer.start(); // starts the timer
    clawSubsystem.setIntake(0.5); // sets the intake speed to .5
  }

  // Called once the command ends or is interrupted.
  @Override // override
  public void end(boolean interrupted) {
    clawSubsystem.disableMotors(); // disables the motors
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(timer.get() > delay) { // if the timer is more than the delay
      return clawSubsystem.getClawMotorCurrent() > currentLimit; // if the clawsubsystem.getcla
    }else { // else
      return false; // returns false
    }
  }
}
