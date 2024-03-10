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

public class ArmSubsystem extends SubsystemBase {
  TalonFX armMotorLeft;
  TalonFX armMotorRight;
  TalonFX extensionMotor;
  DigitalInput topArmLimit;
  DigitalInput bottomArmLimit;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    armMotorLeft = new TalonFX(62);
    armMotorRight = new TalonFX(59);
    extensionMotor = new TalonFX(61);

    armMotorLeft.setNeutralMode(NeutralModeValue.Brake);
    armMotorRight.setNeutralMode(NeutralModeValue.Brake);
    extensionMotor.setNeutralMode(NeutralModeValue.Brake);
    
    topArmLimit = new DigitalInput(3);
    bottomArmLimit = new DigitalInput(4);
  }

  public void setArmSpeed(double speed) {
    armMotorLeft.set(speed);
    armMotorRight.set(-speed);
  }

  public void setExtensionSpeed(double speed){
    extensionMotor.set(speed);
  }

  public void setToPosition(double position){
    //armMotorLeft.set(ControlMode.Position, position);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public double getArmMotor1Position(){
    return armMotorLeft.getPosition().getValueAsDouble() / Constants.armGearRatio * (2 * Math.PI);
  }

  public double getArmMotor2Position() {
    return armMotorRight.getPosition().getValueAsDouble() / Constants.armGearRatio * (2 * Math.PI);
  }

  public boolean getTopLimit(){
    //return topArmLimit.get();
    return false;
  }

  public boolean getBottomLimit(){
    //return bottomArmLimit.get();
    return false;
  }
}
