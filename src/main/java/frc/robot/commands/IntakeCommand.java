// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClawSubsystem;

public class IntakeCommand extends Command {
  private final ClawSubsystem clawSubsystem;

  private final double delay;

  public IntakeCommand(ClawSubsystem clawSubsystem, double speed, double delay) {
    this.clawSubsystem = clawSubsystem; // the subsystem
    this.delay = delay; // the delay
    
    addRequirements(clawSubsystem); // adds the requirements 
  }

  @Override
  public void initialize() { // initialiazes
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
    /*try {
      if(clawSubsystem.getDistanceSensor() < 5.0) { // if the timer is more than the delay
        return true;
      }else { // else
        return false; // returns false
      }
    } catch (Exception e) {
      return false;
    }
  }*/
  return false;
}
}
