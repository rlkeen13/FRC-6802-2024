// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MovementValues;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShoulderSubsystem;

public class AutoScore extends Command {
  /** Creates a new AutoScore. */
  ShooterSubsystem shooter;
  ShoulderSubsystem shoulder;
  double velocity;
  public AutoScore(ShooterSubsystem shooter, ShoulderSubsystem shoulder, double velocity) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.velocity = velocity;
    this.shoulder = shoulder;
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.spinShooterVelocityCommand(velocity, shoulder.instantScoringPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.spinShooterVelocityCommand(0, false);
    shooter.spinIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(shooter.getIntakeSensor() < MovementValues.intakeSensorThreshold && shooter.hasSpun){
      shooter.hasSpun = false;
      return true;
    }
    return false;
  }
}
