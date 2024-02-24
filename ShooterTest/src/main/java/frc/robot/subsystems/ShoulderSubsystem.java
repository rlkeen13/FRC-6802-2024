// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.hardware.TalonFX;

import static edu.wpi.first.units.MutableMeasure.mutable;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.RotationsPerSecond;
import static edu.wpi.first.units.Units.Volts;

import java.util.Map;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.units.Angle;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

public class ShoulderSubsystem extends PIDSubsystem  {
   // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
 // Mutable holder for unit-safe voltage values, persisted to avoid reallocation.
 private final MutableMeasure<Voltage> m_appliedVoltage = mutable(Volts.of(0));
 // Mutable holder for unit-safe linear distance values, persisted to avoid reallocation.
 private final MutableMeasure<Angle> m_angle = mutable(Rotations.of(0));
 // Mutable holder for unit-safe linear velocity values, persisted to avoid reallocation.
 private final MutableMeasure<Velocity<Angle>> m_velocity = mutable(RotationsPerSecond.of(0));

 private ArmFeedforward feedforward = new ArmFeedforward(.37093, 0.30107, 0.0034909, .00022637);

 public boolean instantScoringPosition = false;
 public double target = .85;

  // Create a motion profile with the given maximum velocity and maximum
  // acceleration constraints for the next setpoint.
  private final TrapezoidProfile m_profile =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(1.75, 0.75));
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();



  
  ShuffleboardTab tab = Shuffleboard.getTab("Max Speed Shoulder");
  private GenericEntry maxShoulderSpeed =
      tab.add("Max Speed Shoulder", .6).withWidget(BuiltInWidgets.kNumberSlider).withProperties(Map.of("min", .6, "max", .9))
         .getEntry();
  /** Creates a new ShoulderSubsystem. */
  TalonFX rightShoulder;
  TalonFX leftShoulder;
  DutyCycleEncoder shoulderEncoder = new DutyCycleEncoder(0);
  Encoder shoulderEncoderRelative = new Encoder(1,2);
  
  final DutyCycleOut m_request = new DutyCycleOut(0);
  SysIdRoutine routine = new SysIdRoutine(
    new SysIdRoutine.Config(),
    new SysIdRoutine.Mechanism((Measure<Voltage> volts) -> {
      this.rotate(volts.in(Volts));
    }, log -> {
                // Record a frame for the shooter motor.
                log.motor("arm")
                    .voltage(
                        m_appliedVoltage.mut_replace(
                            rightShoulder.get() * RobotController.getBatteryVoltage(), Volts))
                    .angularPosition(m_angle.mut_replace(shoulderEncoder.getAbsolutePosition(), Rotations))
                    .angularVelocity(
                        m_velocity.mut_replace(shoulderEncoderRelative.getRate(), RotationsPerSecond));
              },
              // Tell SysId to make generated commands require this subsystem, suffix test state in
              // WPILog with this subsystem's name ("shooter")
              this));
  public ShoulderSubsystem() {super(
        new PIDController(
            1.23,
            0,
            .0214));
    rightShoulder = new TalonFX(51);
    rightShoulder.setInverted(false);
    leftShoulder = new TalonFX(52);
    leftShoulder.setControl(new Follower(rightShoulder.getDeviceID(), true));


    this.enable();
    
  }


public Command spinShoulderCommand(double power) {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          rotate(power);
        });
  }


  public void rotate(double speed){
    double rotationPosition = shoulderEncoder.getAbsolutePosition();
    if(rotationPosition <= .89 && rotationPosition >= .55){
      rightShoulder.setVoltage(speed);

      System.out.println(speed);
    }
    else{
      rightShoulder.setVoltage(speed);

      System.out.println(speed);
    }
  }

  public void setShootingAngle(){
    setTargetSetpoint(target, false);
  }

  public void setTargetSetpoint(double target, boolean instantScoringPosition){
    this.instantScoringPosition = instantScoringPosition;
    this.target = target;
  }

  public void disablePid(){
    this.disable();
  }


  @Override
  protected double getMeasurement() {
    // TODO Auto-generated method stub
    return shoulderEncoder.getAbsolutePosition();
  }


  /**
   * Returns a command that will execute a quasistatic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return routine.quasistatic(direction);
  }

  /**
   * Returns a command that will execute a dynamic test in the given direction.
   *
   * @param direction The direction (forward or reverse) to run the test in
   */
  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return routine.dynamic(direction);
  }

  @Override
  protected void useOutput(double output, double setpoint) {
    // TODO Auto-generated method stub
    rightShoulder.set(output * -1);
  }

  @Override
  public void periodic(){    
    //target = maxShoulderSpeed.getDouble(.6);
    this.setSetpoint(target);
    super.periodic();
  }
}
