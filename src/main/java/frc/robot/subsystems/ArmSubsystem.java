package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ArmSubsystem extends SubsystemBase {
    TalonFX leftArmMotor;
    TalonFX rightArmMotor;
    TalonFX extensionMotor;

    public ArmSubsystem() {
        leftArmMotor = new TalonFX(8);
        rightArmMotor = new TalonFX(9);
        extensionMotor = new TalonFX(10);

    }

    // sets speed for the extension motor
    public void setExtensionMotor(double speed) {
        extensionMotor.set(speed);
    }
}