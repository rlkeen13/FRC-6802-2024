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
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDSubsystem;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.MovementValues;

import org.apache.commons.math3.analysis.UnivariateFunction;
import org.apache.commons.math3.analysis.differentiation.UnivariateDifferentiableFunction;
import org.apache.commons.math3.analysis.solvers.*;

//import org.apache.commons.
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
 public double distanceFromLimelightToGoalInches;

  // Create a motion profile with the given maximum velocity and maximum
  // acceleration constraints for the next setpoint.
  private final TrapezoidProfile m_profile =
      new TrapezoidProfile(new TrapezoidProfile.Constraints(1.75, 0.75));
  private TrapezoidProfile.State m_goal = new TrapezoidProfile.State();
  private TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  // how many degrees back is your limelight rotated from perfectly vertical?
  double limelightMountAngleDegrees = 38.3; 

  // distance from the center of the Limelight lens to the floor
  double limelightLensHeightInches = 6; 

  // distance from the target to the floor
  double goalHeightInches = 80.5; 
  double angleToGoalDegrees;
  double angleToGoalRadians;

  
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
            3.56,
            0,
            .0214));
    rightShoulder = new TalonFX(51);
    rightShoulder.setInverted(false);
    leftShoulder = new TalonFX(52);
    leftShoulder.setControl(new Follower(rightShoulder.getDeviceID(), true));

    this.enable();
    
  }

public void shoulderTest(){
  target = maxShoulderSpeed.getDouble(.6);
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

  public void experimentalArmSet(){
    double dist_x_init = distanceFromLimelightToGoalInches;

    // starting height of point from floor
    double y_init = 16.25;
    // r from r,theta of interest point
    double r = 23.25;
    // theta from r,theta of interest point
    double theta_c = Math.atan(5.25/23.75);
    // height to shoot ring at
    double target_height = 81;
    // initial guess if using something like Newton-Raphson
    double init_guess = Math.toRadians(175);
    // starting encoder rotations
    double init_arm_rotations = 0.6;

    var adjacent = distanceFromLimelightToGoalInches + 24;
    var opposite = 80.5 - 12;

    double suggestedAngle = Math.toDegrees(Math.tan(opposite/adjacent));
    SmartDashboard.putNumber("Suggested Arm Angle", suggestedAngle);

    double suggestedArmPosition = (61.0 - suggestedAngle)/365.0 + .6;

    SmartDashboard.putNumber("Suggested Arm Value", suggestedArmPosition);

    // the actual solving part
    // UnivariateFunction height = theta_arm -> Math.tan(theta_arm - theta_c - Math.PI + Math.toRadians(15)) * (dist_x_init + r*Math.cos(theta_arm-theta_c)) + y_init + r*Math.sin(theta_arm-theta_c) - target_height;
    // UnivariateSolver solver = new BrentSolver();
    // double arm_angle = solver.solve(10, height, -Math.PI, Math.PI);

    // convert to usable with encoders
    // double arm_rotations = init_arm_rotations + (-Math.PI - arm_angle)/(2*Math.PI);
    // SmartDashboard.putNumber("arm rotations", arm_rotations);
  }


  @Override
  public double getMeasurement() {
    // TODO Auto-generated method stub
    SmartDashboard.putNumber("arm angle", shoulderEncoder.getAbsolutePosition());
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
    
  
  NetworkTableEntry ty = table.getEntry("ty");
  double targetOffsetAngle_Vertical = ty.getDouble(0.0);
  
    
  angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
  angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

  //calculate distance
  distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);
  SmartDashboard.putNumber("Distance to target", distanceFromLimelightToGoalInches);
  experimentalArmSet();
    if(target == MovementValues.armAway){
      target = maxShoulderSpeed.getDouble(.6);
    }  
    //target 
    this.setSetpoint(target);
    super.periodic();
  }
}
