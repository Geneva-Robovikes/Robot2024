// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class ClimbSubsystem extends SubsystemBase {
  TalonFX leftClimb;
  TalonFX rightClimb;
  PIDController climbPID = new PIDController(1, 0, 0);
  //DigitalInput topArmLimit;
  //DigitalInput bottomArmLimit;

  /** Creates a new ArmSubsystem. */
  public ClimbSubsystem() {
    leftClimb = new TalonFX(71);
    rightClimb = new TalonFX(72);

    leftClimb.setNeutralMode(NeutralModeValue.Brake);
    rightClimb.setNeutralMode(NeutralModeValue.Brake);
  }

  public void setClimbVolts(double voltage) {
    leftClimb.set(voltage);
    rightClimb.set(voltage);
  }

  public void setClimbPosition(double position) {
    double voltage = climbPID.calculate(getClimbPosition(), position);
    leftClimb.setVoltage(voltage);
    rightClimb.setVoltage(voltage);
  }

  
  public double getClimbPosition() {
    return (leftClimb.getRotorPosition().getValueAsDouble() + rightClimb.getRotorPosition().getValueAsDouble()) / 2 / 10;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
