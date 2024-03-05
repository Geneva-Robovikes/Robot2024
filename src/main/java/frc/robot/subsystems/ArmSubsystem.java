// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
  TalonFX falcon1;
  TalonFX falcon2;
  /** Creates a new ArmSubsystem. */
  public ArmSubsystem() {
    falcon1 = new TalonFX(61);

  }

  public void setspeed(double speed) {
    falcon1.set(speed);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
