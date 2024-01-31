// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.common.Gains;
import frc.robot.common.MotorUtils;

public class TalonShooterSubsystem extends SubsystemBase {

  // Talon info
  private static final double talonMaximumTicksPerSecond = Constants.talonMaximumRevolutionsPerMinute * Constants.CtreTalonFx500EncoderTicksPerRevolution / 60;
  private static final double velocitySufficientWarmupThreshold = 0.8;

  // Shooter gearing - currently 1:1
  private static final double topShooterGearRatio = 1.0;
  
  private static final double kMinDeadband = 0.001;
  private static final int kPIDLoopIdx = 0;
  private static final int kTimeoutMs = 30;

  private WPI_TalonFX leftMotor = new WPI_TalonFX(Constants.leftTalonShooterMotorCanId);
//  private WPI_TalonFX rightMotor = new WPI_TalonFX(Constants.rightTalonShooterMotorCanId);

  private Gains leftMotorGains = new Gains(0.50, 0.001, 5, 1023/20660.0, 300, 1.00);
//  private Gains rightMotorGains = new Gains(0.50, 0.001, 5, 1023/20660.0, 300, 1.00);

  public TalonShooterSubsystem() {

    leftMotor.configFactoryDefault();
    leftMotor.setNeutralMode(NeutralMode.Coast);
    leftMotor.setInverted(Constants.leftTalonShooterMotorDefaultDirection);
    leftMotor.configNeutralDeadband(TalonShooterSubsystem.kMinDeadband);
    leftMotor.configSelectedFeedbackSensor(
      TalonFXFeedbackDevice.IntegratedSensor,
      TalonShooterSubsystem.kPIDLoopIdx,
      TalonShooterSubsystem.kTimeoutMs);

    leftMotor.configNominalOutputForward(0, TalonShooterSubsystem.kTimeoutMs);
    leftMotor.configNominalOutputReverse(0, TalonShooterSubsystem.kTimeoutMs);
    leftMotor.configPeakOutputForward(1.0, TalonShooterSubsystem.kTimeoutMs);
    leftMotor.configPeakOutputReverse(-1.0, TalonShooterSubsystem.kTimeoutMs);

    leftMotor.config_kF(TalonShooterSubsystem.kPIDLoopIdx, this.leftMotorGains.kF, TalonShooterSubsystem.kTimeoutMs);
    leftMotor.config_kP(TalonShooterSubsystem.kPIDLoopIdx, this.leftMotorGains.kP, TalonShooterSubsystem.kTimeoutMs);
    leftMotor.config_kI(TalonShooterSubsystem.kPIDLoopIdx, this.leftMotorGains.kI, TalonShooterSubsystem.kTimeoutMs);
    leftMotor.config_kD(TalonShooterSubsystem.kPIDLoopIdx, this.leftMotorGains.kD, TalonShooterSubsystem.kTimeoutMs);    
/*
    rightMotor.configFactoryDefault();
    rightMotor.setNeutralMode(NeutralMode.Coast);
    rightMotor.setInverted(Constants.rightTalonShooterMotorDefaultDirection);
    rightMotor.configNeutralDeadband(TalonShooterSubsystem.kMinDeadband);
    rightMotor.configSelectedFeedbackSensor(
      TalonFXFeedbackDevice.IntegratedSensor,
      TalonShooterSubsystem.kPIDLoopIdx,
      TalonShooterSubsystem.kTimeoutMs);

    rightMotor.configNominalOutputForward(0, TalonShooterSubsystem.kTimeoutMs);
    rightMotor.configNominalOutputReverse(0, TalonShooterSubsystem.kTimeoutMs);
    rightMotor.configPeakOutputForward(1.0, TalonShooterSubsystem.kTimeoutMs);
    rightMotor.configPeakOutputReverse(-1.0, TalonShooterSubsystem.kTimeoutMs);

    rightMotor.config_kF(TalonShooterSubsystem.kPIDLoopIdx, this.rightMotorGains.kF, TalonShooterSubsystem.kTimeoutMs);
    rightMotor.config_kP(TalonShooterSubsystem.kPIDLoopIdx, this.rightMotorGains.kP, TalonShooterSubsystem.kTimeoutMs);
    rightMotor.config_kI(TalonShooterSubsystem.kPIDLoopIdx, this.rightMotorGains.kI, TalonShooterSubsystem.kTimeoutMs);
    rightMotor.config_kD(TalonShooterSubsystem.kPIDLoopIdx, this.rightMotorGains.kD, TalonShooterSubsystem.kTimeoutMs);    
*/
    CommandScheduler.getInstance().registerSubsystem(this);
  }

  /**
   * Set the top shooter motor to a specific velocity using the in-built PID controller
   * @param revolutionsPerMinute - the RPM that the top motor should spin
   */
  public void setShooterVelocityLeft(double revolutionsPerMinute)
  {
    leftMotor.set(
      ControlMode.Velocity,
      this.convertShooterRpmToMotorUnitsPer100Ms(revolutionsPerMinute, TalonShooterSubsystem.topShooterGearRatio));
  }

  /**
   * Set the top shooter motor to a specific velocity using the in-built PID controller
   * @param revolutionsPerMinute - the RPM that the top motor should spin
   */
  public void setShooterVelocityRight(double revolutionsPerMinute)
  {
    /* 
    rightMotor.set(
      ControlMode.Velocity,
      this.convertShooterRpmToMotorUnitsPer100Ms(revolutionsPerMinute, TalonShooterSubsystem.topShooterGearRatio));
      */
  }

  public void setAllStop() {
    this.setShooterSpeedLeft(0.0);
    this.setShooterSpeedRight(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void setShooterSpeedLeft(double speed) {
    leftMotor.set(ControlMode.PercentOutput, speed);
  }

  public void setShooterSpeedRight(double speed) {
//    rightMotor.set(ControlMode.PercentOutput, speed);
  }

  private double convertShooterRpmToMotorUnitsPer100Ms(double targetRpm, double targetGearRatio)
  {
    double targetUnitsPer100ms = 
      MotorUtils.truncateValue(
        targetRpm,
        Constants.talonMaximumRevolutionsPerMinute * -1.0,
        Constants.talonMaximumRevolutionsPerMinute) *
      Constants.CtreTalonFx500EncoderTicksPerRevolution *
      targetGearRatio / 600.0;
    return targetUnitsPer100ms;
  }

}
