// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class armsubsytem extends SubsystemBase {
  TalonFX mainpivotmotor;
  TalonFX intakepivotmotor;
  TalonFX extension1motor;
  TalonFX extension2motor;
  DigitalInput mainpivot;
  DigitalInput intakepivot;
  DigitalInput extensionoutlimit;
  DigitalInput extensioninlimit;
  /** Creates a new armsubsytem. */
  public armsubsytem() {
    mainpivotmotor = new TalonFX(61);
    intakepivotmotor = new TalonFX(62);
    extension1motor = new TalonFX(63);
    extension2motor = new TalonFX(64);
    mainpivot = new DigitalInput(1);
    intakepivot = new DigitalInput(2);
    extensionoutlimit = new DigitalInput(3);
    extensioninlimit = new DigitalInput(4);
  }
public void setfalcon1(double speed){
  mainpivotmotor.set(speed);
}

public void setfalcon2(double speed){
  intakepivotmotor.set(speed);
}

public void setfalcon3(double speed){
  extension1motor.set(speed);
}

public void setfalcon4(double speed){
  extension2motor.set(speed);
}

public boolean getlimit1() {
  return mainpivot.get();
}

public boolean getlimit2() {
  return intakepivot.get();
}

public boolean getlimit3() {
  return extensionoutlimit.get();
}

public boolean getlimit4() {
  return extensioninlimit.get();
}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
