// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ArmSubsystem extends SubsystemBase {
  TalonFX armMotor1;
  TalonFX armMotor2;
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    armMotor1 = new TalonFX(62);
    armMotor2 = new TalonFX(63);
    armMotor1.setNeutralMode(NeutralModeValue.Brake);
    armMotor2.setNeutralMode(NeutralModeValue.Brake);

  }

  public void setspeed(double speed) {
    armMotor1.set(speed);
    armMotor2.set(-speed);

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
}
