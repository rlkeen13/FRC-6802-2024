// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.MovementValues;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberSpin extends Command {
  ClimberSubsystem climber;
  double direction;
  boolean moved = false;
  /** Creates a new ClimberSpin. */
  public ClimberSpin(ClimberSubsystem climber, double direction) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;
    this.direction = direction;
    addRequirements(climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(climber.getSpeed() > .1){
      moved = true;
      System.out.println("moved");
    }
    climber.climbersMove(direction);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    System.out.println(climber.getSpeed());
    if((direction == MovementValues.climberUp && climber.getClimberForwardLimit().getValue()) || 
    (direction == MovementValues.climberDown && climber.getClimberReverseLimit().getValue())){
      climber.resetStickyFaults();
      return true;
    }
    else
      return false;
  }
}
