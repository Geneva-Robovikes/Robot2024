// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  TalonFX falcon1;
  TalonFX falcon2;
  TalonFX falcon3;
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    falcon1 = new TalonFX(61);
    falcon2 = new TalonFX(62);
    falcon3 = new TalonFX(63);
  }

  public void setspeed(double speed) {
    falcon1.set(speed);
    falcon2.set(speed);
    falcon3.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
