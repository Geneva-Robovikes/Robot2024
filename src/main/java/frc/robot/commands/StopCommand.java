package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class StopCommand extends CommandBase  {
    DriveSubsystem driveSubsystem;

    public StopCommand(DriveSubsystem drive) {
        driveSubsystem = drive;
        
    }

    public void execute(){
        driveSubsystem.stop();
    }
}
