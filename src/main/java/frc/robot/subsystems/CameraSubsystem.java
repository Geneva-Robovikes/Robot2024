// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.Constants;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonTargetSortMode;
import org.photonvision.PhotonUtils;
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
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class CameraSubsystem extends SubsystemBase {
  AprilTagFieldLayout tagLayout;
  PhotonCamera camera;

  Transform3d cameraPosition;

  PhotonPipelineResult result;

  PhotonPoseEstimator poseEstimator;
  int speakerTargetId;
  int ampTargetId;
  Optional<Alliance> ally = DriverStation.getAlliance();


  /**
   * Creates a new camera with name, position, and pitch from the horizontal.
   * @param cameraName name of the camera on Photon Vision Dashboard.
   * @param cameraPosition Position of the camera from robot center.
   * @param cameraPitch
   */
  public CameraSubsystem(String name, Transform3d cameraPosition) {
    camera = new PhotonCamera(name);
    camera.setPipelineIndex(0);
    result = camera.getLatestResult();
   this.cameraPosition = cameraPosition;

    if (ally.isPresent()){

      if(ally.get() == Alliance.Red){
        speakerTargetId = 4;
        ampTargetId = 5;
      }

      if(ally.get() == Alliance.Blue){
        speakerTargetId = 7;
        ampTargetId = 6;
      }
    }
    


    try {
      tagLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.kDefaultField.m_resourceFile);
    } catch (Exception e) {
      System.err.println("File cannot be loaded");
    }
    poseEstimator = new PhotonPoseEstimator(tagLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, camera, cameraPosition);
  }

  @Override
  public void periodic() {
    //System.out.println(camera.getLatestResult());
    result = camera.getLatestResult();
 
  }

  /**
   * Get whether the camera sees targets.
   * @return true if targets are found.
   */
  public boolean hasTargets() {
    return result.hasTargets();
    //result.getBestTarget().getSkew();
  }

  /**
   * Get the best target's skew.
   * @return Skew angle.
   */
  //added by alex
  public double getTarget1Skew() {
    if (hasTargets()){
      return result.getBestTarget().getSkew();
    }
    else return 0.0;
  }

  public double getTarget1Yaw() {
    if (hasTargets()){
      return result.getBestTarget().getYaw();
    }
    else return 0.0;
  }

  public double getTarget1Pitch() {
    if (hasTargets()){
      return result.getBestTarget().getPitch();
    }
    else return 0.0;
  }

  public double getTargetDistance(){
    if (hasTargets()){
      return PhotonUtils.calculateDistanceToTargetMeters(0, 0, 0, getTarget1Pitch());
    }
    else return 0.0;
    
  }

  public int getSpeakerTargetId(){
    return speakerTargetId;
  }

  public int getAmpTargetId(){
    return ampTargetId;
  }

  public int getTarget1Ids(){
    if (hasTargets()){
      return result.getBestTarget().getFiducialId();
    }
    else return -1;
  }

  public Transform3d getLocation1(){
    if (result.getMultiTagResult().estimatedPose.isPresent) {
      return result.getMultiTagResult().estimatedPose.best;
    }
    return null;
    
  
  }


  /*public pose getTargetPose() {
    result.getBestTarget().getDistanceToPose();
  }*/

  /**
   * Sets the camera's  to the index's corresponding pipeline.
   * @return The index of the pipeline. 0 = April Tag, 1 = Disk
   */
  public int getPipelineIndex() {
    return camera.getPipelineIndex();
  }

  /**
   * Sets the camera's pipeline to the index's corresponding pipeline.
   * @param index The index of the pipeline. 0 = April Tag, 1 = Disk
   */
  public void setPipeline(int index) {
    camera.setPipelineIndex(index);
  }

  /**
   * Gets the estimated pose from an apriltag.
   * @param prevEstimatedRobotPose The current pose in the odometry.
   * @return The new pose estimate.
   */
  public Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
    poseEstimator.setReferencePose(prevEstimatedRobotPose);
    return poseEstimator.update();
  }
}