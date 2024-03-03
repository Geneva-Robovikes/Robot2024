// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.CameraSubsystem;

public class CameraController extends SubsystemBase {
  /** Creates a new CameraController. */
  CameraSubsystem camera1;
  CameraSubsystem camera2;

  public CameraController() {
    camera1 = new CameraSubsystem("camera 1", null);
    camera2 = new CameraSubsystem("camera 2", null);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
