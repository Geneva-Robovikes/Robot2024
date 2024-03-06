// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawPivotSubsystem extends SubsystemBase {
 static TalonFX falcon1;
 DigitalInput limitswitch;
  /** Creates a new Clawpivotsubsystem. */
  public ClawPivotSubsystem() {
    falcon1 = new TalonFX(63);
    limitswitch = new DigitalInput(1);
  }
public void setSpeed(double speed) {
  falcon1.set(speed);
}
public boolean getlimit(){
  return limitswitch.get();
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
