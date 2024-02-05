package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class StopCommand extends Command  {
    DriveSubsystem driveSubsystem;

    public StopCommand(DriveSubsystem drive) {
        driveSubsystem = drive;
<<<<<<< HEAD
=======
        
>>>>>>> 1b59b3fe9800500206f98bef2074d5351bdf99cb
        addRequirements(drive);
    }

    public void execute(){
        driveSubsystem.stop();
    }
}
