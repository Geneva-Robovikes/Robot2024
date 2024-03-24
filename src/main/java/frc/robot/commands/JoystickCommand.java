// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawPivotSubsystem;
import frc.robot.Constants;

public class JoystickCommand extends Command {
  private final ClawPivotSubsystem clawPivotSubsystem;
  private final ArmSubsystem armSubsystem;

  private final CommandXboxController controller;

  /** Creates a new JoystickCommand. */
  public JoystickCommand(CommandXboxController controller, ArmSubsystem armSubsystem, ClawPivotSubsystem clawpivotsubsystem) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.controller = controller;
    this.armSubsystem = armSubsystem;
    this.clawPivotSubsystem = clawpivotsubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double leftY = controller.getLeftY();
    double rightY = controller.getRightY();
  

    if(((leftY < 0 ) || (leftY > 0 )) && Math.abs(leftY) > Constants.controllerDeadzone){
      armSubsystem.setArmSpeed(leftY * Constants.maxArmSpeed);
    } else armSubsystem.setArmSpeed(0);

    if (((rightY < 0 && !clawPivotSubsystem.getLimit()) || (rightY > 0)) && Math.abs(rightY) > Constants.controllerDeadzone) {
      clawPivotSubsystem.setSpeed(rightY * Constants.maxPivotSpeed);
    } else clawPivotSubsystem.setSpeed(0);



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
