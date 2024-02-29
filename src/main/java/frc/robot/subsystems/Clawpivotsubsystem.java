// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Clawpivotsubsystem extends SubsystemBase {
  static TalonFX falcon1;
  /** Creates a new Clawpivotsubsystem. */
  public Clawpivotsubsystem() {
    falcon1 = new TalonFX(71);
  }
public static void setspeed(double speed) {
  falcon1.set(speed);
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
