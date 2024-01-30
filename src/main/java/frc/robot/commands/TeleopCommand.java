package frc.robot.commands;

import java.io.Console;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.DriveSubsystem;

public class TeleopCommand extends CommandBase {
    private final DriveSubsystem driveSubsystem;
    private final CommandXboxController driveController;

    private final double maxSpeedXY = Constants.maxTranlationalDriveSpeed;
    private final double maxSpeedTheta = Constants.maxRotationalDriveSpeed;
    boolean isFieldCentric = true;

    public TeleopCommand(DriveSubsystem driveSubsystem, CommandXboxController driveController){
        this.driveSubsystem = driveSubsystem;
        this.driveController = driveController;
        addRequirements(driveSubsystem);
    }

    public void execute() {
        // Squares inputs so fine movements are easier to make
        double x1 = Math.signum(driveController.getLeftX()) * Math.pow(driveController.getLeftX(), 2);
        double y1 = Math.signum(-driveController.getLeftY()) * Math.pow(driveController.getLeftY(), 2);
        double x2 = Math.signum(-driveController.getRightX()) * Math.pow(driveController.getRightX(), 2);
        double rightTrigger = driveController.getRightTriggerAxis();

        // If the right controller is pressed, drive switches to robot centric
        if(rightTrigger > 0.5) {
            isFieldCentric = false;
        } else {
            isFieldCentric = true;
        }

        //controlls to reset gyro
        if(driveController.rightTrigger().getAsBoolean()) {
            driveSubsystem.resetGyro();
        }

        // Applies a deadzone to the controller inpts
        x1 = MathUtil.applyDeadband(x1, OperatorConstants.controllerDeadzone);
        y1 = MathUtil.applyDeadband(y1, OperatorConstants.controllerDeadzone);
        x2 = MathUtil.applyDeadband(x2, OperatorConstants.controllerDeadzone);

        // Switches inputs to speeds in meters per second
        double vX = y1 * maxSpeedXY;
        double vY = x1 * maxSpeedXY;
        double vTheta = x2 * maxSpeedTheta;

        // Sets the drive speeds
        driveSubsystem.setModuleStatesFromSpeeds(vX, vY, vTheta, isFieldCentric);
    }

    public void end(boolean interrupted) {
        driveSubsystem.setModuleStatesFromSpeeds(0, 0, 0, isFieldCentric);
    }

    public boolean isFinished() {
        return false;
    }
}
