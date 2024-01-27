package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc.robot.Constants;


public class SwerveModule {
    TalonFX driveMotor;
    TalonFX turnMotor;

    //TODO add pid stuff
    ProfiledPIDController drivePID = new ProfiledPIDController(0, 0, 0, new Constraints(Constants.maxModuleVelocity, Constants.maxModuleVelocity));
    PIDController turnPID = new PIDController(0, 0, 0);

    SimpleMotorFeedforward drivFeedforward = new SimpleMotorFeedforward(0, 0, 0);
    SimpleMotorFeedforward turnFeedforward = new SimpleMotorFeedforward(0, 0, 0);


    public SwerveModule(int driveMotorIndex, int turnMotorIndex, boolean driveInterted, boolean turnInverted){
        driveMotor = new TalonFX(driveMotorIndex);
        turnMotor = new TalonFX(turnMotorIndex);

        driveMotor.setInverted(driveInterted);
        turnMotor.setInverted(turnInverted);
        resetModule();
        turnMotor.setNeutralMode(NeutralModeValue.Brake);
        driveMotor.setNeutralMode(NeutralModeValue.Brake);
        

    }

    public void stopModule(){
        driveMotor.setVoltage(0);
        turnMotor.setVoltage(0);
    }

    public void setModule(double driveVolts, double turnVolts){
        driveMotor.setVoltage(driveVolts);
        turnMotor.setVoltage(turnVolts);
    }

    public SwerveModulePosition getPosition(){
        return new SwerveModulePosition(
            getDrivenDistance(),
            new Rotation2d(getCurrentAngle())
            );
            
        
    }

    public void resetModule(){
        driveMotor.setPosition(0);
        turnMotor.setPosition(0);
        
    }

    public void setDesiredState(SwerveModuleState desiredState){
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(getCurrentAngle()));

        double driveOutput = drivePID.calculate(getDriveVelocity(), state.speedMetersPerSecond);
        double driveFeed = drivFeedforward.calculate(state.speedMetersPerSecond);
        double turnOutput = turnPID.calculate(getCurrentAngle(), state.angle.getRadians());

        if(Math.abs(driveOutput + driveFeed) > .75){
            driveMotor.setVoltage(driveOutput + driveFeed);
        }

        else{
            driveMotor.setVoltage(0);
        }

        if (Math.abs(turnOutput) > .75){
            turnMotor.setVoltage(turnOutput);
        }

        else{
            turnMotor.setVoltage(0);
        }
    }

    private double getDriveVelocity(){
        return driveMotor.getPosition().getValue() / Constants.swerveDriveGearRatio /Constants.falconEncoderResolution * Math.PI * Constants.swerveWheelDiameter;
    }

    private double getDrivenDistance(){
        return driveMotor.getPosition().getValue() / Constants.swerveDriveGearRatio /Constants.falconEncoderResolution * Math.PI * Constants.swerveWheelDiameter;

    }

    private double getCurrentAngle(){
        return turnMotor.getPosition().getValue() / Constants.swerveTurnGearRatio / Constants.falconEncoderResolution * 2 * Math.PI;
    }




    
}
