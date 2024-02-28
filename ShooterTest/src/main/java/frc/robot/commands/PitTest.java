// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants.MovementValues;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PitTest extends SequentialCommandGroup {
  /** Creates a new PitTest. */
  public PitTest(SwerveSubsystem swerve, ClimberSubsystem climber, ShooterSubsystem shooter, ShoulderSubsystem shoulder) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ClimberSpin(climber, MovementValues.climberUp), 
    new WaitCommand(2),
    new ClimberSpin(climber, MovementValues.climberDown),
    new WaitCommand(2),
    new AutoCollectionCommand(shooter, shoulder),
    new WaitCommand(2),
    new ShoulderMoveCommand(shoulder, MovementValues.armUp, false),
    new WaitCommand(2),
    new AutoScore(shooter, shoulder, MovementValues.defaultVelocity),
    new AutoCollectionCommand(shooter, shoulder),
    new WaitCommand(2),
    new ShoulderMoveCommand(shoulder, MovementValues.defaultScore, true),
    new AutoScore(shooter, shoulder, MovementValues.defaultVelocity), 
    swerve.driveCommand(()-> 0.0, ()->0.5, ()->0.0).withTimeout(5),
    new WaitCommand(2),
    swerve.driveCommand(()-> .5, ()->0.0, ()->0.0).withTimeout(5)  
    );
  }
}
