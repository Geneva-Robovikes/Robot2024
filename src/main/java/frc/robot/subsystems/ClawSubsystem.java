// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClawSubsystem extends SubsystemBase {
  private final CANSparkMax intake;
  private final CANSparkMax outtake1;
  private final CANSparkMax outtake2;
  //intake can is 53, shoot is 52 (nNEEDS TO BE NEGATIVE), shoot2 is 51 (POSITIVE)
  /** Creates a new IntakeSubsystem. */
  public ClawSubsystem() {
    intake = new CANSparkMax(53, MotorType.kBrushless);
    outtake1 = new CANSparkMax(52, MotorType.kBrushless);
    outtake2  = new CANSparkMax(51, MotorType.kBrushless);

    intake.setIdleMode(IdleMode.kBrake);
    outtake1.setIdleMode(IdleMode.kBrake);
    outtake2.setIdleMode(IdleMode.kBrake);

    outtake2.setInverted(true);
    intake.setInverted(true);
  }

  public void setOuttake(){
    outtake1.set(.95);
    outtake2.set(.95);
  }
//outake needs more fine tuneing for power levels.
  public void setIntake(double speed){
    intake.set(speed);
  }

  public void disableMotors(){
    System.out.println("okay disable motors");

    intake.set(0);
    outtake1.set(0);
    outtake2.set(0);
  }

  public double getClawMotorCurrent() {
    return intake.getOutputCurrent();
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
