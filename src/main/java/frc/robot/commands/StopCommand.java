package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;


public class StopCommand extends Command  {
    private final DriveSubsystem driveSubsystem;

    public StopCommand(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;

        addRequirements(driveSubsystem);
    }

    public void execute(){
        driveSubsystem.stop();
    }
}
