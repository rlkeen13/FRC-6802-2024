// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class AutoIntakeCommand extends Command {
  ShooterSubsystem shooter;
  /** Creates a new AutoIntakeCommand. */
  public AutoIntakeCommand(ShooterSubsystem shooter) {
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.spinIntake(-.5);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.spinIntake(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(shooter.getIntakeSensor() > 3){
      System.out.println("Finshed: ");
      System.out.println(shooter.getIntakeSensor());
      return true;
    }
    return false;
  }
}
