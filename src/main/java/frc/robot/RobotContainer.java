// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TeleopCommand;
import frc.robot.commands.JoystickCommand;
import frc.robot.commands.PresetCommand;
import frc.robot.commands.AmpShootCommand;
import frc.robot.commands.AutoForwardsCommand;
import frc.robot.commands.ExtentionCommand;
import frc.robot.commands.IntakeCommand;

import frc.robot.subsystems.ClawPivotSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.DriveSubsystem;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;



import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

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
  public final ClawPivotSubsystem clawPivotSubsystem = new ClawPivotSubsystem();
  public final DriveSubsystem driveSubsystem = new DriveSubsystem();
  public final ClawSubsystem clawSubsystem = new ClawSubsystem();
  public final ArmSubsystem armSubsystem = new ArmSubsystem();

  
  /* ~~~~Commands~~~~ */
  public final IntakeCommand intakeCommand = new IntakeCommand(clawSubsystem, -.5, .80);
  public final ShootCommand shootCommand = new ShootCommand(clawSubsystem, 1);
  public final AmpShootCommand ampShootCommand = new AmpShootCommand(clawSubsystem);

  /* ~~~~Presets~~~~ */
  public final PresetCommand presetCommand = new PresetCommand(clawSubsystem, armSubsystem, clawPivotSubsystem, 0);


  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController = new CommandXboxController(
      OperatorConstants.kDriverControllerPort);
  private final CommandXboxController controllController = new CommandXboxController(
    OperatorConstants.kControlControllerPort);


  //private final CommandXboxController controlController = new CommandXboxController(
  //    OperatorConstants.kControlControllerPort);
  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */

  SendableChooser<String> autoChooser = new SendableChooser<>();

  public RobotContainer() {
    autoChooser.setDefaultOption("Test Auto", "Test Auto");
    autoChooser.addOption("Forwards", "Forwards");
    
    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Configure the trigger bindings
    configureBindings();
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
     controllController.leftBumper().whileTrue(ampShootCommand);
     controllController.rightTrigger().whileTrue(intakeCommand);
     controllController.leftTrigger().whileTrue(shootCommand);

    
    controllController.povDown().whileTrue(new ExtentionCommand(armSubsystem, 0));
    controllController.povUp().whileTrue(new ExtentionCommand(armSubsystem, 1));

    controllController.a().whileTrue(presetCommand);
   /* 
   controllController.rightTrigger().whileTrue(intakeCommand);
   controllController.leftTrigger().whileTrue(shootCommand);
   controllController.leftBumper().whileTrue(ampShootCommand);

   controllController.povDown().whileTrue(new ExtentionCommand(armSubsystem, 0));
   controllController.povUp().whileTrue(new ExtentionCommand(armSubsystem, 1));
   */


   //clawController.povUp().whileTrue(ArmupCommand);
   //clawController.povDown().whileTrue(ArmdownCommand);
   //clawController.povRight().whileTrue(clawpivotupcommand);
   //clawController.povLeft().whileTrue(clawpivotdowncommand);



  }

  public Command getTeleopCommand() {
    return new ParallelCommandGroup(
    new TeleopCommand(driveSubsystem, driverController),
    new JoystickCommand(controllController, armSubsystem, clawPivotSubsystem)
    );
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  
  public Command getAutonomousCommand() {

    if(autoChooser.getSelected().equals("Forwards")) {
      return new ParallelCommandGroup(
      new AutoForwardsCommand(driveSubsystem, -0.6, 4.25, true));
    }
    
    PathPlannerPath testPath = PathPlannerPath.fromPathFile("Test Path");
    return AutoBuilder.followPath(testPath);
  }
  
}
