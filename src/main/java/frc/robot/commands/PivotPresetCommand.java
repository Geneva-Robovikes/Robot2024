// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawPivotSubsystem;
import frc.robot.subsystems.ClawSubsystem;


public class PivotPresetCommand extends Command {
  private final ClawPivotSubsystem clawPivotSubsystem;
  private final Timer timer = new Timer();

  private double goal;
  private double Kp;


  public PivotPresetCommand(ClawPivotSubsystem clawPivotSubsystem, double goal) {
    this.clawPivotSubsystem = clawPivotSubsystem;
    Kp = .013;
    this.goal = goal;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer.reset();
    timer.start();
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  /*  if ((clawPivotSubsystem.getPosition() < (goal -= 2.0)) ) {
      clawPivotSubsystem.setSpeed(-.3);
    } 
    if((clawPivotSubsystem.getPosition() > (goal += 2.0)) ) {
      clawPivotSubsystem.setSpeed(.3);
    }*/

    /*
    double currentPosition = clawPivotSubsystem.getPosition();
    double error = goal - currentPosition;
    double command = error * Kp;
    clawPivotSubsystem.setSpeed(-command);
    */

    clawPivotSubsystem.setClawAngle(goal);  

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    clawPivotSubsystem.setSpeed(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (/*clawPivotSubsystem.getPosition() == goal) ||*/ timer.get() > 3);
  }
}
