// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.TeleopCommand;
import frc.robot.commands.JoystickCommand;
import frc.robot.commands.PivotPresetCommand;
import frc.robot.commands.AmpShootCommand;
import frc.robot.commands.AutoForwardsCommand;
import frc.robot.commands.ClimbDownCommand;
import frc.robot.commands.ClimbUpCommand;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.commands.ExtentionCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.IntakeOppositeCommand;
import frc.robot.subsystems.ClawPivotSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.ClawSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.commands.ArmPresetCommand;
import frc.robot.commands.CameraAlignCommand;
import frc.robot.commands.AutoShootCommand;
import frc.robot.commands.AutoIntake;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import edu.wpi.first.math.geometry.Transform3d;
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
  private final SendableChooser<Command> autoChooser;

  /* ~~~Subsystems~~~ */
  public final ClawPivotSubsystem clawPivotSubsystem = new ClawPivotSubsystem();
  public final DriveSubsystem driveSubsystem = new DriveSubsystem();
  public final ClawSubsystem clawSubsystem = new ClawSubsystem();
  public final ArmSubsystem armSubsystem = new ArmSubsystem();
  public final ClimbSubsystem climbSubsystem = new ClimbSubsystem();
 

  
  /* ~~~~Commands~~~~ */
  public final IntakeOppositeCommand intakeOppositeCommand = new IntakeOppositeCommand(clawSubsystem);
  public final AmpShootCommand ampShootCommand = new AmpShootCommand(clawSubsystem);
  public final IntakeCommand intakeCommand = new IntakeCommand(clawSubsystem, -.5, .30);
  public final ShootCommand shootCommand = new ShootCommand(clawSubsystem, 3.5);
  public final AutoShootCommand autoShootCommand = new AutoShootCommand(clawSubsystem, 3.5);
  public final ClimbDownCommand climbDownCommand = new ClimbDownCommand(climbSubsystem);
  public final ClimbUpCommand climbUpCommand = new ClimbUpCommand(climbSubsystem);
  public final AutoIntake autoIntake = new AutoIntake(clawSubsystem, 3);



  /* ~~~~Presets~~~~ */
  public final PivotPresetCommand pivotUpPresetCommand = new PivotPresetCommand(clawPivotSubsystem, -91.0);
  public final PivotPresetCommand pivotUpPresetLongCommand = new PivotPresetCommand(clawPivotSubsystem, -71.0);
  public final PivotPresetCommand pivotDownPresetCommand = new PivotPresetCommand(clawPivotSubsystem, 0);
  public final ArmPresetCommand ampArmCommand = new ArmPresetCommand(armSubsystem, 0);
  public final PivotPresetCommand ampPivotCommand = new PivotPresetCommand(clawPivotSubsystem, 0);
  public final PivotPresetCommand pivotIntakePresetCommand = new PivotPresetCommand(clawPivotSubsystem, -138.0);


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

  

  public RobotContainer() {
    NamedCommands.registerCommand("Auto Claw Pivot Up", pivotUpPresetCommand);
    NamedCommands.registerCommand("Auto Claw Pivot Up Long", pivotUpPresetLongCommand);
    NamedCommands.registerCommand("Auto Claw Pivot Down", pivotDownPresetCommand);
    NamedCommands.registerCommand("Shoot", autoShootCommand);
    NamedCommands.registerCommand("intake", autoIntake);
    NamedCommands.registerCommand("intake position", pivotIntakePresetCommand);

    //NamedCommands.registerCommand("auto shoot", );
    autoChooser = AutoBuilder.buildAutoChooser();
   // ShuffleboardContainer.add
    SmartDashboard.putData("Auto Chooser", autoChooser);


    // Configure the trigger bindings
    configureBindings();
  }

  public void encoderTest() {
        SmartDashboard.putNumber("Gyro", driveSubsystem.getGyroAngleY());
        SmartDashboard.putNumber("Pivot Position", clawPivotSubsystem.getPosition());
        SmartDashboard.putNumber("Arm1 Positon", armSubsystem.getArmMotor1Position());
        SmartDashboard.putNumber("Arm2 Position", armSubsystem.getArmMotor2Position());
        //SmartDashboard.putNumber("target pitch", cameraSubsystem.getTarget1Pitch());
        //SmartDashboard.putNumber("target yaw", cameraSubsystem.getTarget1Yaw());
        //SmartDashboard.putNumber("target id", cameraSubsystem.getTarget1Ids());

        SmartDashboard.putBoolean("limit", clawPivotSubsystem.getLimit());

    

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
    controllController.rightBumper().whileTrue(intakeOppositeCommand);
    
    controllController.rightTrigger().whileTrue(intakeCommand);
    controllController.leftTrigger().whileTrue(shootCommand);

    controllController.povUp().whileTrue(climbUpCommand);
    controllController.povDown().whileTrue(climbDownCommand);   
   
    
    //controllController.b().whileTrue(cameraAlignCommand);

    controllController.a().whileTrue(pivotUpPresetCommand);
    //controllController.b().whileTrue(ampArmCommand.andThen(ampPivotCommand));
    controllController.y().whileTrue(pivotDownPresetCommand);
    controllController.x().whileTrue(pivotIntakePresetCommand);
    controllController.b().whileTrue(pivotUpPresetLongCommand);
    controllController.povUp().whileTrue(climbUpCommand);
    controllController.povDown().whileTrue(climbDownCommand);
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
   * Use this to pass the autonomous command to the main {@link Robot} cla
   * ]]]]]]]]]]]]]]]]]]]]]]]]ss.
   *
   * @return the command to run in autonomous
   */
  
  public Command getAutonomousCommand() {
    
    return autoChooser.getSelected();

  /*  if(autoChooser.getSelected().equals("Forwards")) {
      return new ParallelCommandGroup(
      new AutoForwardsCommand(driveSubsystem, -0.6, 4.25, true));
    }
    
    PathPlannerPath testPath = PathPlannerPath.fromPathFile("Test Path");
    return AutoBuilder.followPath(testPath);
    */
  }
  
}