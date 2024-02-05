package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class StopCommand extends Command  {
    DriveSubsystem driveSubsystem;

    public StopCommand(DriveSubsystem drive) {
        driveSubsystem = drive;
        addRequirements(drive);
    }

    public void execute(){
        driveSubsystem.stop();
    }
}
