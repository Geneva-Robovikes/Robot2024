// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.Volts;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig; 


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry; // imports edu.wpi.first.math.kinematics.SwerveDriveOdometry
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.commands.StopCommand;

public class DriveSubsystem extends SubsystemBase {

  ADIS16448_IMU gyro = new ADIS16448_IMU();

  //Positions are based of of 25in square robot
  Translation2d frontLeftLocation = new Translation2d(0.3048, 0.3048);
  Translation2d frontRightLocation = new Translation2d(0.3048, -0.3048);
  Translation2d backLeftLocation = new Translation2d(-0.3048, 0.3048);
  Translation2d backRightLocation = new Translation2d(-0.3048, -0.3048);

  //Establishes motor indecies for each module.
  SwerveModule frontRightModule = new SwerveModule(41, 42, 43, Constants.frontRightEncoderOffset, false, true, "front right");
  SwerveModule frontLeftModule = new SwerveModule(31,32, 33, Constants.frontLeftEncoderOffset, false, true, "front left");
  public SwerveModule backRightModule = new SwerveModule(12, 11, 13, Constants.backRightEncoderOffset, false, true, "back right");
  SwerveModule backLeftModule = new SwerveModule(22, 21, 23, Constants.backLeftEncoderOffset, false, true, "back left");
  

  //Creates the drive kinematics. This is the math for moving the drivetrain.
  public SwerveDriveKinematics kinematics = new SwerveDriveKinematics(frontLeftLocation, frontRightLocation, backLeftLocation, backRightLocation);

  //Creates the drive odometry. This calculates the position of the robot on the field.
  SwerveDriveOdometry odometry = new SwerveDriveOdometry(
    kinematics, getRotation2d(), new SwerveModulePosition[] {
      frontLeftModule.getPosition(),
      frontRightModule.getPosition(),
      backLeftModule.getPosition(), 
      backRightModule.getPosition()
    }, 
    new Pose2d(0, 0, new Rotation2d())
  );

  /**
   * Creates the drivetrain. This contains methods for driving.
   */
  public DriveSubsystem() {
    gyro.calibrate();
    setDefaultCommand(new StopCommand(this));

    //configures auto
    AutoBuilder.configureHolonomic(
      this::getPose, 
      this::resetOdometry, 
      this::getRobotChassisSpeeds, 
      this::driveRobotRelative, 
      new HolonomicPathFollowerConfig(
        new PIDConstants(5, 0, 0),
        new PIDConstants(5, 0, 0),
        4.5,
        0.3048,
        new ReplanningConfig()
      ), 
      () -> {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
          return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
      }, 
      this);
  }
  
  //Updates the odometry's position of the swerve drive on the field.
  @Override
  public void periodic() {
    odometry.update(getRotation2d(),
    new SwerveModulePosition[] {
      frontLeftModule.getPosition(), frontRightModule.getPosition(),
      backLeftModule.getPosition(), backRightModule.getPosition()
    });
  }

  /**
   * Sets the speed of the drive on the field.
   * In robot centric drive, forward on the X joystick is the robots forward.
   * In field centric drive, forward on the X joysick is away from the driver station.
   * All directions correspond to this pattern.
   * @param xVelocity Sets the speed of the drive in the X direction. Measured in meters per second.
   * @param yVelocity Sets the speed of the drive in the Y direction. Measured in meters per second.
   * @param angularVelocity Sets the angular speed of the drive. Measured in radians per second.
   * @param isFieldCentric Decides whether the drive is field centric or robot centric.
   */
  public void setModuleStatesFromSpeeds(double xVelocity, double yVelocity, double angularVelocity, boolean isFieldCentric) {
    ChassisSpeeds speeds;

    if(isFieldCentric) {
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(xVelocity, -yVelocity, angularVelocity, getRotation2d());
    } else {
      speeds = new ChassisSpeeds(xVelocity, -yVelocity, -angularVelocity);
    }
    SwerveModuleState[] states = kinematics.toSwerveModuleStates(speeds);
    //Set max speed/max velocity here
    SwerveDriveKinematics.desaturateWheelSpeeds(states, 3);
    setModuleStates(states);
  }

  private ChassisSpeeds getRobotChassisSpeeds() {
    return kinematics.toChassisSpeeds(getModuleStates());
  }

  private void driveRobotRelative(ChassisSpeeds robotRelativeSpeeds) {
    ChassisSpeeds targetSpeeds = ChassisSpeeds.discretize(robotRelativeSpeeds, 0.02);
    SwerveModuleState[] targetStates = kinematics.toSwerveModuleStates(targetSpeeds);
    setModuleStates(targetStates);
  }

  /**
   * Resets the gyro's rotation.
   */
  public void resetGyro() {
    gyro.reset();
  }

  /*
   *  In theory aligns swerve.
   */
  public void swerveAlignment() {
  }
  /**
   * Resets the odometry's zero position the current position.
   * @param pose The new pose in meters and radians.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(getRotation2d(), 
      new SwerveModulePosition[] {
        frontLeftModule.getPosition(),
        frontRightModule.getPosition(),
        backLeftModule.getPosition(), 
        backRightModule.getPosition()
      }, pose
    );
  }

  /**
   * Returns the current robot odometry position.
   * @return the current pose.
   */
  public Pose2d getPose() {
    return odometry.getPoseMeters();
  }

  /**
   * Returns the current rotation from the gyro.
   * @return the current rotation in radians.
   */
  public Rotation2d getRotation2d() {
    return new Rotation2d(-gyro.getGyroAngleZ() / 57.295779513);
  }


  /**
   * Returns the gyro Y angle.
   * @return the angle in degrees.
   */
  public double getGyroAngleY() {
    return gyro.getGyroAngleY();
  }

  /**
   * Returns the gyro Y rate.
   * @return the rate in degrees per second.
   */
  public double getGyroRateY() {
    return gyro.getGyroRateY();
  }

  /**
   * Sets each module to its specified state. Must be in the order:
   * [frontLeft, frontRight, backLeft, backRight]
   * @param moduleStates the module states to so set.
   */
  public void setModuleStates(SwerveModuleState[] moduleStates) {
    frontLeftModule.setDesiredState(moduleStates[0]);
    frontRightModule.setDesiredState(moduleStates[1]);
    backLeftModule.setDesiredState(moduleStates[2]);
    backRightModule.setDesiredState(moduleStates[3]);
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = {
      frontLeftModule.getModuleState(), frontRightModule.getModuleState(),
      backLeftModule.getModuleState(), backRightModule.getModuleState()
    };
    return states;
  }

  /**
   * Stops the swerve modules.
   */
  public void stop() {
    frontLeftModule.stopModule();
    frontRightModule.stopModule();
    backLeftModule.stopModule();
    backRightModule.stopModule();
  }

  /**
   * Sets all modules to the specivied values.
   * @param driveVolts the drive voltage. Must be in range -12.0 to 12.0
   * @param turnVolts the turn voltage. Must be in range -12.0 to 12.0
   */
  public void setModules(double driveVolts, double turnVolts) {
    frontLeftModule.setModule(driveVolts, turnVolts);
    frontRightModule.setModule(driveVolts, turnVolts);
    backLeftModule.setModule(driveVolts, turnVolts);
    backRightModule.setModule(driveVolts, turnVolts);
  }

  // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
  private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
  // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
  private final MutableMeasure<Distance> m_distance = mutable(Meters.of(0));
  // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
  private final MutableMeasure<Velocity<Distance>> m_velocity = mutable(MetersPerSecond.of(0));

  SysIdRoutine routine = new SysIdRoutine(
    new SysIdRoutine.Config(),
    new SysIdRoutine.Mechanism(
      (Measure<Voltage> volts) -> {
        setModules(volts.in(Volts), 0);
      },
      log -> {
        log.motor("Front Left").voltage(m_appliedVoltage.mut_replace(backRightModule.getDriveVoltage(), Volts));
        log.motor("Front Left").linearPosition(m_distance.mut_replace(backRightModule.getDriveDistance(), Meters));
        log.motor("Front Left").linearVelocity(m_velocity.mut_replace(backRightModule.getDriveVelocity(), MetersPerSecond));
      },
      this)
  );

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }

  public Command turnSysIdQuasistatic(SysIdRoutine.Direction direction) {
    return backRightModule.turnRoutine.quasistatic(direction);
  }

  public Command turnSysIdDynamic(SysIdRoutine.Direction direction) {
    return backRightModule.turnRoutine.dynamic(direction);
  }

}