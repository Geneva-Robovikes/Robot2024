package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;

public class StopCommand  {
    DriveSubsystem driveSubsystem;

    public StopCommand(DriveSubsystem drive) {
        driveSubsystem = drive;
        
    }

    public void execute(){
        driveSubsystem.stop();
    }
}
