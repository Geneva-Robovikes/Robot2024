// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TeleopCommand;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  /* ~~~Subsystems~~~ */
  public final DriveSubsystem driveSubsystem = new DriveSubsystem();
  public final ClawSubsystem clawSubsystem = new ClawSubsystem();
  
  /* ~~~~Commands~~~~ */
  public final IntakeCommand intakeCommand = new IntakeCommand(clawSubsystem, -.5, 32, .80);
  public final ShootCommand shootCommand = new ShootCommand(clawSubsystem, 1);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  private final CommandXboxController clawController = new CommandXboxController(1);
  //private final CommandXboxController controlController = new CommandXboxController(
  //    OperatorConstants.kControlControllerPort);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  SendableChooser<String> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    autoChooser.setDefaultOption("Test Auto", "Test Auto");
    
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure the trigger bindings
    configureBindings();
  }

  public void checkLimitSwitch() {

  }

  public void encoderTest() {
        SmartDashboard.putNumber("Gyro", driveSubsystem.getGyroAngleY());
    

  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    /*
     * new Trigger(m_exampleSubsystem::exampleCondition)
     * .onTrue(new ExampleCommand(m_exampleSubsystem));
     * 
     * // Schedule `exampleMethodCommand` when the Xbox controller's B button is
     * pressed,
     * // cancelling on release.
     * m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());
     */

   //driverController.a().whileTrue(driveSubsystem.backRightModule.turnTest());
   //driverController.a().whileTrue(driveSubsystem.turnSysIdDynamic(SysIdRoutine.Direction.kForward));
   //driverController.b().whileTrue(driveSubsystem.turnSysIdDynamic(SysIdRoutine.Direction.kReverse));
   //driverController.x().whileTrue(driveSubsystem.turnSysIdQuasistatic(SysIdRoutine.Direction.kForward));
   //driverController.y().whileTrue(driveSubsystem.turnSysIdQuasistatic(SysIdRoutine.Direction.kReverse));

   clawController.rightTrigger().whileTrue(intakeCommand);
   clawController.leftTrigger().whileTrue(shootCommand);
  }

  public Command getTeleopCommand() {
    return new TeleopCommand(driveSubsystem, driverController);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    PathPlannerPath testPath = PathPlannerPath.fromPathFile("Test Path");
    return AutoBuilder.followPath(testPath);
  }
}
