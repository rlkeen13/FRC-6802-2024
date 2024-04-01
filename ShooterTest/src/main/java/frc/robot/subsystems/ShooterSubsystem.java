// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMMotorController;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.motorcontrol.PWMTalonSRX;
import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.MovementValues;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  CANSparkMax topMotor;
  CANSparkMax bottomMotor;
  VictorSP intake;
  private SparkPIDController topPidController;
  private SparkPIDController bottomPidController;
  AnalogInput intakeSensor = new AnalogInput(3);
  ShuffleboardTab tab = Shuffleboard.getTab("Max Speed Shooter");
  private GenericEntry maxSpeedShooter =
      tab.add("Max Speed Shooter", 4000).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", 1000, "max", 5000))
         .getEntry();
  private GenericEntry maxSpeedIntake =
      tab.add("Max Speed Intake", 0).withWidget(BuiltInWidgets.kNumberSlider)
         .getEntry();
         
  public double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM;
  public boolean hasSpun = false;
  public ShooterSubsystem() {

    topMotor = new CANSparkMax(61, MotorType.kBrushless);
    bottomMotor = new CANSparkMax(62, MotorType.kBrushless);
    intake = new VictorSP(9);
    topPidController = topMotor.getPIDController();
    bottomPidController = bottomMotor.getPIDController();

    kP = 0.00001; 
    kI = 0;
    kD = 0; 
    kIz = 0; 
    kFF = 0.00019; 
    kMaxOutput = 1; 
    kMinOutput = -1;
    maxRPM = 5700;

      // display PID coefficients on SmartDashboard

    

    

    // set PID coefficients
    topPidController.setP(kP, 0);
    topPidController.setI(kI, 0);
    topPidController.setD(kD, 0);
    topPidController.setIZone(kIz, 0);
    topPidController.setFF(kFF, 0);
    topPidController.setOutputRange(kMinOutput, kMaxOutput, 0);

      SmartDashboard.putNumber("P Gain", topPidController.getP());
      SmartDashboard.putNumber("I Gain", kI);
      SmartDashboard.putNumber("D Gain", kD);
      SmartDashboard.putNumber("I Zone", kIz);
      SmartDashboard.putNumber("Feed Forward", topPidController.getFF());
      SmartDashboard.putNumber("Max Output", kMaxOutput);
      SmartDashboard.putNumber("Min Output", kMinOutput);
      SmartDashboard.putNumber("Target Speed", 3000);

    // set PID coefficients
    bottomPidController.setP(kP, 0);
    bottomPidController.setI(kI, 0);
    bottomPidController.setD(kD, 0);
    bottomPidController.setIZone(kIz, 0);
    bottomPidController.setFF(kFF, 0);
    bottomPidController.setOutputRange(kMinOutput, kMaxOutput, 0);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command exampleMethodCommand() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
        });
  }
  public Command spinShooterCommand(double power) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          spinShooter(power);
        });
  }
  public Command spinIntakeCommand(double power) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runEnd(
        () -> {
          if(intakeSensor.getVoltage() <1.5){
            spinIntake(power);
          }
          else{
            spinIntake(0);
          }
        }, ()-> {spinIntake(0);});
  }

  public void spinShooterVelocityCommand(double velocity, boolean instantScoring){
      // if(velocity != 0)
      //   velocity = maxSpeedShooter.getDouble(4000.0);
      double actualVelo = topMotor.getEncoder().getVelocity();
      double bottomVelo = bottomMotor.getEncoder().getVelocity();
      SmartDashboard.putNumber("Top Wheels", actualVelo);
      SmartDashboard.putNumber("Bottom Wheels", bottomVelo);
      double error = velocity - Math.abs(actualVelo);
      if(error < 100 || instantScoring){
        hasSpun = true;
        spinIntake(MovementValues.intakeInShoot);
      }

    topPidController.setReference(velocity, ControlType.kVelocity);
    bottomPidController.setReference(velocity, ControlType.kVelocity);
      System.out.println(error);
}

  public void spinShooter(double power){
    topMotor.set(power);
    bottomMotor.set(-power);
    spinIntake(power);
  }

  public void spinShooterVelocity(double velocity){
  }

  public void spinIntake(double power){
      intake.set(power);
  }

  public double getIntakeSensor(){
    return intakeSensor.getVoltage();
  }

  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    //spinIntake(maxSpeedIntake.getDouble(0));
    //spinShooter(maxSpeedShooter.getDouble(0));
  // System.out.println(intakeSensor.getVoltage());
   double p = SmartDashboard.getNumber("P Gain", 0);
    double ff = SmartDashboard.getNumber("Feed Forward", 0);

    // if PID coefficients on SmartDashboard have changed, write new values to controller
    if((p != kP)) { topPidController.setP(p);bottomPidController.setP(p); kP = p; }
    if((ff != kFF)) { topPidController.setFF(ff); bottomPidController.setFF(ff);kFF = ff; }

  }
  

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
