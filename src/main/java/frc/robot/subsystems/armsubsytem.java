// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class armsubsytem extends SubsystemBase {
  TalonFX falcon1;
  TalonFX falcon2;
  TalonFX falcon3;
  TalonFX falcon4;
  DigitalInput limitswitch;
  /** Creates a new armsubsytem. */
  public armsubsytem() {
    falcon1 = new TalonFX(61);
    falcon2 = new TalonFX(62);
    falcon3 = new TalonFX(63);
    falcon4 = new TalonFX(64);
    limitswitch = new DigitalInput(1);
  }
public void setfalcon1(double speed){
  falcon1.set(speed);
}

public void setfalcon2(double speed){
  falcon2.set(speed);
}

public void setfalcon3(double speed){
  falcon3.set(speed);
}

public void setfalcon4(double speed){
  falcon4.set(speed);
}

public boolean getlimit() {
  return limitswitch.get();
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
