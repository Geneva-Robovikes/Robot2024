// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj.DigitalInput;

public class ArmSubsystem extends SubsystemBase {
  TalonFX armMotor1;
  TalonFX armMotor2;
  TalonFX extensionMotor;
  DigitalInput topArmLimit;
  DigitalInput bottomArmLimit;

  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    armMotor1 = new TalonFX(62);
    armMotor2 = new TalonFX(63);
    extensionMotor = new TalonFX(61);
    armMotor1.setNeutralMode(NeutralModeValue.Brake);
    armMotor2.setNeutralMode(NeutralModeValue.Brake);
    extensionMotor.setNeutralMode(NeutralModeValue.Brake);
    topArmLimit = new DigitalInput(3);
    bottomArmLimit = new DigitalInput(4);
  }

  public void setArmSpeed(double speed) {
    armMotor1.set(speed);
    armMotor2.set(-speed);
  }

  public void setExtensionSpeed(double speed){
    extensionMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public double getArmMotor1Angle(){
    return armMotor1.getPosition().getValueAsDouble() * (2 * Math.PI);
  }

  public double getArmMotor2Angle() {
    return armMotor2.getPosition().getValueAsDouble() * (2 * Math.PI);
  }

  public boolean getTopLimit(){
    return topArmLimit.get();
  }

  public boolean getBottomLimit(){
    return bottomArmLimit.get();
  }
}
