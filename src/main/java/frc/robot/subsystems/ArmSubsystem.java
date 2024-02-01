package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.ADIS16448_IMU;
import frc.robot.commands.StopCommand;

public ArmSubsystem extends SubsystemBase{
    TalonFX leftArmMotor;
    TalonFX rightArmMotor;
    TalonFX extensionMotor;

    public ArmSubsystem(){
        leftArmMotor = new TalonFX(8);
        rightArmMotor = new TalonFX(9);
        extensionMotor new TalonFX(10);


        leftArmMotor.setNeutralMode(neutralMode.Brake);
        rightArmMotor.setNuetralMode(neutralMode.Brake);
    }

    //sets speed for the extension motor
    public void setExtensionMotor()
}