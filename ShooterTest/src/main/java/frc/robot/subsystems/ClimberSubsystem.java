// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new climbersystem. */
  TalonFX rightClimber = new TalonFX(35);
  TalonFX leftClimber = new TalonFX(36); 
  SoftwareLimitSwitchConfigs limitSwitchConfig = new SoftwareLimitSwitchConfigs();
  MotorOutputConfigs motorOutputConfigs = new MotorOutputConfigs();
  public ClimberSubsystem() 
  {
    motorOutputConfigs.NeutralMode = NeutralModeValue.Brake;
    limitSwitchConfig.ForwardSoftLimitEnable = true;
    limitSwitchConfig.ReverseSoftLimitEnable = true;
    limitSwitchConfig.ForwardSoftLimitThreshold = 23;
    limitSwitchConfig.ReverseSoftLimitThreshold = 3;
    rightClimber.getConfigurator().apply(limitSwitchConfig);
    leftClimber.getConfigurator().apply(limitSwitchConfig);
    rightClimber.getConfigurator().apply(motorOutputConfigs);
    leftClimber.getConfigurator().apply(motorOutputConfigs);
  }

  public void climbersUp(){
    rightClimber.set(1);
    leftClimber.set(1);
  }
  public void climbersDown(){
    rightClimber.set(-1);
    leftClimber.set(-1);
  }
  public StatusSignal<Boolean> getClimberReverseLimit(){
    return rightClimber.getStickyFault_ReverseSoftLimit();
  }

  public StatusSignal<Boolean> getClimberForwardLimit(){
    return rightClimber.getStickyFault_ForwardSoftLimit();
  }

  public void resetStickyFaults(){
    rightClimber.clearStickyFault_ForwardSoftLimit();
    rightClimber.clearStickyFault_ReverseSoftLimit();
  }

  public void climbersMove(double speed){
    rightClimber.set(speed);
    leftClimber.set(speed);
  }

  public double getSpeed(){
   return Math.abs(rightClimber.getAcceleration().getValueAsDouble());
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
