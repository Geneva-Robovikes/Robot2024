package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

public class SwerveModule extends SubsystemBase {
    TalonFX driveMotor;
    TalonFX turnMotor;
    CANcoder encoder;
    double offset;
    String moduleName;
    SwerveModuleState currentState;

    //PID controllers allow for accurate position/velocity tracking
    //Profiled PID controller is an extension of PID controllers that allows for velocity and acceleration constraints
    //These are feedback controllers, so they correct for error
    ProfiledPIDController drivePID = new ProfiledPIDController(0, 0, 0, new Constraints(Constants.maxModuleVelocity, Constants.maxModuleAcceleration));
    //PIDController turnPID = new PIDController(6.1807, 0, 0.23405);
    PIDController turnPID = new PIDController(2.3815, .0, 0);
    //ki 0.027342 0.48255

    //Feedforward controllers anticipate motion
    SimpleMotorFeedforward driveFeedForward = new SimpleMotorFeedforward(-0.095829, 2.7601, 0.71108);
    SimpleMotorFeedforward turnFeedForward = new SimpleMotorFeedforward(0.24959, 0.3754, 0.0068821);

    /**
     * Creates a swerve module!
     * @param driveMotorIndex The index of the driving motor
     * @param turnMotorIndex The index of the turning motor
     * @param driveInverted True if the drive motor should be inverted
     * @param turnInverted True if the turn motor should be inverted
     */
    public SwerveModule(int driveMotorIndex, int turnMotorIndex, int encoderIndex, double encoderOffset, boolean driveInverted, boolean turnInverted, String moduleName) {
        driveMotor = new TalonFX(driveMotorIndex);
        turnMotor = new TalonFX(turnMotorIndex);
        encoder = new CANcoder(encoderIndex);
        offset = encoderOffset;
        this.moduleName = moduleName;
        
        turnPID.enableContinuousInput(-Math.PI, Math.PI);
        driveMotor.setInverted(driveInverted);
        turnMotor.setInverted(turnInverted);
        
        encoder.getConfigurator().apply(new CANcoderConfiguration());
        
        //System.out.println("Error: " + (encoder.getAbsolutePosition().getValueAsDouble() - encoderOffset));
        
        driveMotor.setNeutralMode(NeutralModeValue.Brake);
        turnMotor.setNeutralMode(NeutralModeValue.Brake);
        
        resetModule();
    }

    /**
     * Sets the drive and turn voltages to zero
     */
    public void stopModule() {
        driveMotor.setVoltage(0);
        turnMotor.setVoltage(0);
    }

    public double getEncoderDegrees() {
        return (encoder.getAbsolutePosition().getValueAsDouble() - offset * 360);
    }

    public double getEncoderRotations() {
        return (encoder.getAbsolutePosition().getValueAsDouble() - offset);
    }

    public Command turnTest() {
        return this.run(() -> turnMotor.setVoltage(4));
    }

    /**
     * Sets the drive and turn motors to the specified speeds
     * @param driveVolts Voltage to set to the drive motor. Must be from -12.0 to 12.0
     * @param turnVolts Voltage to set to the turn motor. Must be from -12.0 to 12.0
     */
    public void setModule(double driveVolts, double turnVolts) {
        driveMotor.setVoltage(driveVolts);
        turnMotor.setVoltage(turnVolts);
    }

    /**
     * Returns the current position of the swerve module.
     * @return A new SwerveModulePosition with the drive motor's distance and the turn motor's angle
     */
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
            getDriveDistance(),
            new Rotation2d(getCurrentAngle())
        );
    }

    public double getDriveVoltage() {
        return driveMotor.get() * RobotController.getBatteryVoltage();
    }

    /**
     * Sets the drive and turn motor's position back to 0.
     */
    public void resetModule() {
        driveMotor.setPosition(0);
        turnMotor.setPosition(0);
    }

    /**
     * Drives the swerve module from the desired state.
     * @param desiredState The state to attempt to reach
     */
    public void setDesiredState(SwerveModuleState desiredState) {
        // SwerveModuleState.optimize() fixes wheel rotation making it more efficient.
        // In simple terms, turning 180 degrees and driving forward is the same as turning 90 degrees and drving backwards.
        SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(getCurrentAngle()));
        currentState = state;
        //state = new SwerveModuleState(state.speedMetersPerSecond, state.angle.div(2));
        SmartDashboard.putNumber(moduleName + " desired state", state.angle.getRadians());

        // The calculate method gets the result from the controllers.
        // For feedback, they take in the current position and the target position.
        // For feedforward, they take in the target position.
        double driveOutput = drivePID.calculate(getDriveVelocity(), state.speedMetersPerSecond);
        //double driveOutput = 0;
        double driveFeed = driveFeedForward.calculate(state.speedMetersPerSecond);
        double turnOutput = turnPID.calculate(getCurrentAngle(), state.angle.getRadians());
        
        
        // The PID controllers will return very small decimals which will leave the motors at a constant stall.
        // This can damage the motors, so this deadzone is a minimum voltage to be returned to drive the module.
        if(Math.abs(driveOutput + driveFeed) > 0.75) {
            //To implement feedback and feedforward controllers add their outputs!
            driveMotor.setVoltage(driveOutput + driveFeed);
        } else {
            driveMotor.setVoltage(0);
        }
        
        // Same idea as above!
        if(Math.abs(turnOutput) > 0.25) {
            turnMotor.setVoltage(turnOutput);
        } else {
            turnMotor.setVoltage(0);
        }
        
    }

    public SwerveModuleState getModuleState() {
        return currentState;
    }

    /**
     * Returns the current wheel velocity of the module.
     * @return The velocity in meters per second.
     */
    public double getDriveVelocity() {
        return driveMotor.getRotorVelocity().getValueAsDouble() / Constants.swerveDriveGearRatio  * Math.PI * Constants.swerveWheelDiameter;
    }

    /**
     * Returns the current wheel position of the module.
     * @return The position of the wheel in meters.
     */
    public double getDriveDistance() {
        return driveMotor.getRotorPosition().getValueAsDouble() / Constants.swerveDriveGearRatio * Math.PI * Constants.swerveWheelDiameter;
    }

    /**
     * Returns the current wheel angle of the module.
     * @return The angle of the wheel in radians.
     */
    public double getCurrentAngle() {
        //SmartDashboard.putNumber(moduleName+"e", (encoder.getPosition().getValueAsDouble()* 2 * Math.PI) - offset);
        //SmartDashboard.putNumber(moduleName+"i", turnMotor.getRotorPosition().getValueAsDouble()/ Constants.swerveTurnGearRatio * 2 * Math.PI);
        //return turnMotor.getRotorPosition().getValueAsDouble() / Constants.swerveTurnGearRatio * 2 * Math.PI;
        return (encoder.getPosition().getValueAsDouble() * 2 * Math.PI) -offset;
        
        
        //return (encoder.getAbsolutePosition().getValueAsDouble() - offset) * 2 * Math.PI;
    }

    public double getAngularVelocity() {
        return encoder.getVelocity().getValueAsDouble() * 2 * Math.PI;
    }

/*
      // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
    private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
    // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
    private final MutableMeasure<Angle> m_distance = mutable(Radians.of(0));
    // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
    private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RadiansPerSecond.of(0));

    public SysIdRoutine turnRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(),
        new SysIdRoutine.Mechanism(
          (Measure<Voltage> volts) -> {
            turnMotor.setVoltage(volts.in(Volts));
          },
          log -> {
              log.motor("Back Right").voltage(m_appliedVoltage.mut_replace(turnMotor.get() * RobotController.getBatteryVoltage(), Volts));
              log.motor("Back Right").angularPosition(m_distance.mut_replace(getCurrentAngle(), Radians));
              log.motor("Back Right").angularVelocity(m_velocity.mut_replace(getAngularVelocity(), RadiansPerSecond));
          },
          this)
        );*/
}