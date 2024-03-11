// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawPivotSubsystem extends SubsystemBase {
  TalonFX falcon1;
  //DigitalInput topLimitSwitch;
  //DigitalInput bottomLimitSwitch;
  /** Creates a new Clawpivotsubsystem. */
  public ClawPivotSubsystem() {
    falcon1 = new TalonFX(10);
    //topLimitSwitch = new DigitalInput(1);
    //bottomLimitSwitch = new DigitalInput(2);
  }
public void setSpeed(double speed) {
  falcon1.set(-speed);
}
/*public boolean getToplimit(){
  //return topLimitSwitch.get();
  return false;
}

public boolean getBottomLimit(){
  //return bottomLimitSwitch.get();
  return false;
}*/

public double getPosition() {
  return falcon1.getPosition().getValueAsDouble();
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
