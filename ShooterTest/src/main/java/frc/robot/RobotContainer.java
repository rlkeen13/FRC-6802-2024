// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.MovementValues;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AutoCollectionCommand;
import frc.robot.commands.AutoIntakeCommand;
import frc.robot.commands.AutoScore;
import frc.robot.commands.PitTest;
import frc.robot.commands.ShoulderMoveCommand;
import frc.robot.commands.AbsoluteDriveAdv;
import frc.robot.subsystems.ClimberSubsystem;
// import frc.robot.commands.Autos;
// import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve"));
  private final ShoulderSubsystem shoulder = new ShoulderSubsystem();
  private final ShooterSubsystem shooter = new ShooterSubsystem();
  private final ClimberSubsystem climber = new ClimberSubsystem();
  private final PitTest pitTest = new PitTest(drivebase, climber, shooter, shoulder);

  private final int translationAxis = 1;
  private final int strafeAxis = 0;
  private final int rotationAxis = 3;

  private final SendableChooser<Command> autoChooser;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  // private final Joystick m_driverController =
  //     new Joystick(OperatorConstants.kDriverControllerPort);
  // JoystickButton intakeButton;
  // JoystickButton shooterButton;
  private final Joystick driverController = new Joystick(0);
  private final Joystick operatorController = new Joystick(1);
  private final JoystickButton shoulderLifButton = new JoystickButton(operatorController, 5);
  private final JoystickButton shoulderLowerButton = new JoystickButton(operatorController, 11);
  private final JoystickButton shoulderScoreButton = new JoystickButton(operatorController, 7);
  private final JoystickButton intakeButton = new JoystickButton(operatorController, 6);
  private final JoystickButton shooterButton = new JoystickButton(operatorController, 13);
  private final JoystickButton sourceScoring = new JoystickButton(operatorController, 4);
  private final JoystickButton shoulderQuasiBackward = new JoystickButton(operatorController, 17);
  private final JoystickButton shoulderStaticForward = new JoystickButton(operatorController, 12);
  private final JoystickButton shoulderStaticBackward = new JoystickButton(operatorController, 18);
  private final JoystickButton climberUp = new JoystickButton(operatorController, 8);
  private final JoystickButton climberDown = new JoystickButton(operatorController, 10);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
     
    // UsbCamera camera = new UsbCamera("cam0", 0);
    // camera.setFPS(30);
    // camera.setResolution(320, 240);
    
    // CameraServer.startAutomaticCapture(camera);
    // UsbCamera camera = CameraServer.startAutomaticCapture();
    // camera.setResolution(360, 240);
    //camera.setFPS(15);

    NamedCommands.registerCommand("autoShoot", new AutoScore(shooter, shoulder, MovementValues.defaultVelocity));
    NamedCommands.registerCommand("autoCollect", new AutoCollectionCommand(shooter, shoulder));
    NamedCommands.registerCommand("autoArmDefault", new ShoulderMoveCommand(shoulder, MovementValues.defaultScore, false));
    NamedCommands.registerCommand("autoArmAway", new ShoulderMoveCommand(shoulder, .71, false));
    NamedCommands.registerCommand("autoArmStraightAway", new ShoulderMoveCommand(shoulder, MovementValues.armAway, false));
    NamedCommands.registerCommand("autoArmCollect", new ShoulderMoveCommand(shoulder, MovementValues.armDown, false));
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);
    // Configure the trigger bindings

    AbsoluteDriveAdv closedAbsoluteDriveAdv = new AbsoluteDriveAdv(drivebase,
                                                                   () -> MathUtil.applyDeadband(driverController.getRawAxis(translationAxis),
                                                                                                OperatorConstants.LEFT_Y_DEADBAND),
                                                                   () -> -MathUtil.applyDeadband(driverController.getRawAxis(strafeAxis),
                                                                                                OperatorConstants.LEFT_X_DEADBAND),
                                                                   () -> MathUtil.applyDeadband(driverController.getRawAxis(rotationAxis),
                                                                                                OperatorConstants.RIGHT_X_DEADBAND),
                                                                   () -> operatorController.getRawButtonPressed(12),
                                                                   () -> operatorController.getRawButtonPressed(15),
                                                                   () -> operatorController.getRawButtonPressed(19),
                                                                   () -> operatorController.getRawButtonPressed(17));


    // SmartDashboard.putNumber("Shooter", 0);
    // SmartDashboard.putNumber("Intake", 0);
    // intakeButton = new JoystickButton(m_driverController, 1);
    // shooterButton = new JoystickButton(m_driverController, 2);
    configureBindings();
    Command closedFieldRel = drivebase.driveCommand(
        () -> (Math.abs(driverController.getRawAxis(translationAxis)) > OperatorConstants.LEFT_Y_DEADBAND) ? -driverController.getRawAxis(translationAxis) : 0,
        () -> (Math.abs(driverController.getRawAxis(strafeAxis)) > OperatorConstants.LEFT_X_DEADBAND) ? -driverController.getRawAxis(strafeAxis) : 0,
        () -> (Math.abs(driverController.getRawAxis(rotationAxis)) > .12 ) ? -driverController.getRawAxis(rotationAxis) : 0);

    Command closedFieldRelSim = drivebase.simDriveCommand(
        () -> (Math.abs(driverController.getRawAxis(translationAxis)) > OperatorConstants.LEFT_Y_DEADBAND) ? driverController.getRawAxis(translationAxis) : 0,
        () -> (Math.abs(driverController.getRawAxis(strafeAxis)) > OperatorConstants.LEFT_X_DEADBAND) ? -driverController.getRawAxis(strafeAxis) : 0,
        () -> (Math.abs(driverController.getRawAxis(rotationAxis)) > .12 ) ? -driverController.getRawAxis(rotationAxis) : 0);

      // Command driveFieldOrientedDirectAngle = drivebase.driveCommand(
      //   () -> MathUtil.applyDeadband(driverController.getRawAxis(translationAxis), OperatorConstants.LEFT_Y_DEADBAND),
      //   () -> MathUtil.applyDeadband(driverController.getRawAxis(strafeAxis), OperatorConstants.LEFT_X_DEADBAND),
      //   () -> driverController2.getRawAxis(0),
      //   () -> driverController2.getRawAxis(1));
        // Command closedFieldRel = drivebase.driveCommand(
    //     () -> 0,
    //     () -> 0,
    //     () -> 0);

    drivebase.setDefaultCommand(!RobotBase.isSimulation() ? closedAbsoluteDriveAdv : closedFieldRelSim);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
   // new JoystickButton(driverController, 3).onTrue((new InstantCommand(drivebase::zeroGyro)));
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // Should be, lower arm and spin intake until cancelled which then raises the arm to the stowed position
    intakeButton.whileTrue(Commands.run(() -> shoulder.setTargetSetpoint(MovementValues.armDown, false), shoulder).alongWith(shooter.spinIntakeCommand(MovementValues.intakeIn)).finallyDo(()->shoulder.setTargetSetpoint(MovementValues.armStow, false)));
    shooterButton.whileTrue(Commands.runEnd(()->shooter.spinShooterVelocityCommand(MovementValues.defaultVelocity, shoulder.instantScoringPosition), ()->{shoulder.setTargetSetpoint(MovementValues.armStow, false); shooter.spinShooterVelocityCommand(0.0, true); shooter.spinIntake(0);}, shoulder));
    
    shoulderLifButton.onTrue(Commands.run(() -> shoulder.setTargetSetpoint(MovementValues.armUp, true), shoulder));
    shoulderLowerButton.onTrue(new ShoulderMoveCommand(shoulder, MovementValues.armAway, false));
    shoulderScoreButton.onTrue(Commands.run(() -> shoulder.setTargetSetpoint(MovementValues.defaultScore, false), shoulder));
    //shoulderScoreButton.onTrue(Commands.run(() -> shoulder.setTargetSetpoint(.72, false), shoulder));
    climberUp.whileTrue(Commands.run(() -> {climber.climbersMove(MovementValues.climberUp); shoulder.setTargetSetpoint(MovementValues.armUp, false);}, climber));
    climberDown.whileTrue(Commands.runEnd(() -> climber.climbersMove(MovementValues.climberDown), () -> climber.climbersMove(0), climber));

    sourceScoring.whileTrue(Commands.run(() -> shoulder.setTargetSetpoint(.78, false), shoulder).alongWith(shooter.spinIntakeCommand(MovementValues.intakeIn)).finallyDo(()->shoulder.setTargetSetpoint(MovementValues.armStow, false)));
    //shoulderQuasiBackward.whileTrue(shoulder.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

    //shoulderStaticForward.whileTrue(shoulder.sysIdDynamic(SysIdRoutine.Direction.kForward));

    //shoulderStaticBackward.whileTrue(shoulder.sysIdDynamic(SysIdRoutine.Direction.kReverse));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();

  }

  public Command getTestCommand(){
    return pitTest;
  }
}
