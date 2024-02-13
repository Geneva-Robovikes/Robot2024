package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DriveSubsystem;

public class AutoDistance extends Command {
    DriveSubsystem driveSubsystem;

    public AutoDistance(DriveSubsystem drive) {
        driveSubsystem = drive;
        driveSubsystem.resetOdometry(new Pose2d());

        addRequirements(drive);
    }
    
    @Override
    public void execute() {
      driveSubsystem.setModules(3, 0);
    }

    @Override
    public boolean isFinished() {
        if(driveSubsystem.getPose().getX() > 3)
            return true;
        return false;
    }
    
    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putNumber("Robot Distance", driveSubsystem.getPose().getX());
    }
}