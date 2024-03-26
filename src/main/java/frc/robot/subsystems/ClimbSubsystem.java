// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClimbSubsystem extends SubsystemBase {

  private final TalonFX climbMotor1;
  private final TalonFX climbMotor2;
  private final DigitalInput climbLimit1Top;
  private final DigitalInput climbLimit2Top;
  private final DigitalInput climbLimit1Bottom;
  private final DigitalInput climbLimit2Bottom;
  /** Creates a new ClimbSubsystem. */
  public ClimbSubsystem() {
    climbMotor1 = new TalonFX(1);
    climbMotor2 = new TalonFX(2);
    climbLimit1Top = new DigitalInput(3);
    climbLimit2Top = new DigitalInput(0);
    climbLimit1Bottom = new DigitalInput(4);
    climbLimit2Bottom = new DigitalInput(1);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public boolean getClimbLimit1Top(){
    //SmartDashboard.putBoolean("radio side top", getClimbLimit1Top());
    return climbLimit1Top.get();
  }

  public boolean getClimbLimit2Top(){
    //SmartDashboard.putBoolean("rio side top", getClimbLimit2Top());
    return climbLimit2Top.get();
  }

  public boolean getClimbLimit1Bottom(){
    //SmartDashboard.putBoolean("radio side bottom", getClimbLimit1Bottom());
    return climbLimit1Bottom.get();
  }

  public boolean getClimbLimit2Bottom(){
    //SmartDashboard.putBoolean("rio side bottom", getClimbLimit2Bottom());    
    return climbLimit2Bottom.get();
  }

  public void setSpeed1(double speed) {
    climbMotor1.set(speed);
  }

  public void setSpeed2(double speed){
    climbMotor2.set(speed);
  }
}
