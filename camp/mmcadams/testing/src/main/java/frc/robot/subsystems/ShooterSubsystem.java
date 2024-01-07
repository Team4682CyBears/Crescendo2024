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

public class ShooterSubsystem extends SubsystemBase {

  // Talon info
  private static final double talonMaximumTicksPerSecond = Constants.talonMaximumRevolutionsPerMinute * Constants.CtreTalonFx500EncoderTicksPerRevolution / 60;
  private static final double velocitySufficientWarmupThreshold = 0.8;

  // Shooter gearing - currently 1:1
  private static final double topShooterGearRatio = 1.0;
  
  private static final double kMinDeadband = 0.001;
  private static final int kPIDLoopIdx = 0;
  private static final int kTimeoutMs = 30;

  private WPI_TalonFX topMotor = new WPI_TalonFX(Constants.shooterMotorCanId);
  private Gains topMotorGains = new Gains(0.1, 0.001, 5, 1023/20660.0, 300, 1.00);

  public ShooterSubsystem() {

    topMotor.configFactoryDefault();
    topMotor.setNeutralMode(NeutralMode.Coast);
    topMotor.setInverted(Constants.shooterTopMotorDefaultDirection);
    topMotor.configNeutralDeadband(ShooterSubsystem.kMinDeadband);
    topMotor.configSelectedFeedbackSensor(
      TalonFXFeedbackDevice.IntegratedSensor,
      ShooterSubsystem.kPIDLoopIdx,
      ShooterSubsystem.kTimeoutMs);

    topMotor.configNominalOutputForward(0, ShooterSubsystem.kTimeoutMs);
    topMotor.configNominalOutputReverse(0, ShooterSubsystem.kTimeoutMs);
    topMotor.configPeakOutputForward(1.0, ShooterSubsystem.kTimeoutMs);
    topMotor.configPeakOutputReverse(-1.0, ShooterSubsystem.kTimeoutMs);

    CommandScheduler.getInstance().registerSubsystem(this);
  }

  /**
   * Set the top shooter motor to a specific velocity using the in-built PID controller
   * @param revolutionsPerMinute - the RPM that the top motor should spin
   */
  public void setShooterVelocityTop(double revolutionsPerMinute)
  {
    System.out.println("Got here!!!");
    topMotor.set(ControlMode.PercentOutput, 0.7);
    /* 
    topMotor.set(
      ControlMode.Velocity,
      this.convertShooterRpmToMotorUnitsPer100Ms(revolutionsPerMinute, ShooterSubsystem.topShooterGearRatio));
      */
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
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
