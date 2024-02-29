// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShoulderSubsystem;

public class ShoulderMoveCommand extends Command {
  /** Creates a new ShoulderMoveCommand. */
  ShoulderSubsystem shoulder;
  double position;
  boolean instantScoring;
  public ShoulderMoveCommand(ShoulderSubsystem shoulder, double position, boolean instantScoring) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shoulder = shoulder;
    this.position = position;
    this.instantScoring = instantScoring;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shoulder.setTargetSetpoint(position, instantScoring);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println("Arm Error: ");
    System.out.println(Math.abs(shoulder.getMeasurement() - position));
    if(Math.abs(shoulder.getMeasurement() - position) < .05 )
      return true;
    else
      return false;
  }
}
