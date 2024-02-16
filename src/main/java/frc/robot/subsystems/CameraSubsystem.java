// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonTargetSortMode;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;

public class CameraSubsystem extends SubsystemBase {


  /** Creates a new CameraSubsystem. */
  
    AprilTagFieldLayout tagLayout;
    PhotonCamera camera;
    Transform3d cameraPosition;
    PhotonPipelineResult result;
    PhotonPoseEstimator poseEstimator;
  

  public CameraSubsystem(String cameraName, Transform3d cameraPosition){
    camera = new PhotonCamera(cameraName);
    camera.setPipelineIndex(3);
    result = camera.getLatestResult();
    this.cameraPosition = cameraPosition;

    try{
      tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
    } catch (Exception e){
      System.err.println("No File");
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    result = camera.getLatestResult();
  }

  public boolean hasTargts(){
    return result.hasTargets();
  }

  public double getTargetSkew(){
    return result.getBestTarget().getSkew();
  }

  public double getTargetPitch(){
    return result.getBestTarget().getPitch();
  }

  public int getPiplineIndex(){
    return camera.getPipelineIndex();
  }

  public void setPipelineIndex(int index){
    camera.setPipelineIndex(index);
  }

  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    poseEstimator.setReferencePose(prevEstimatedRobotPose);
    return poseEstimator.update();
  }
}
