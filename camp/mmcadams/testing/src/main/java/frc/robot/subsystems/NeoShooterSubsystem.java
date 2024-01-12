// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.common.Gains;

public class NeoShooterSubsystem extends SubsystemBase {

  // for example code see: https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Smart%20Motion%20Example/vendordeps/REVLib.json

  // Shooter gearing - currently 1:1
  private static final double topShooterGearRatio = 1.0;
  private static final double kMinDeadband = 0.001;
  private static final int kPIDLoopIdx = 0;
  private static final int kTimeoutMs = 30;

  private CANSparkMax leftMotorOne = new CANSparkMax(Constants.leftNeoOneShooterMotorCanId, MotorType.kBrushless);
  private CANSparkMax leftMotorTwo = new CANSparkMax(Constants.leftNeoTwoShooterMotorCanId, MotorType.kBrushless);
  private CANSparkMax rightMotorOne = new CANSparkMax(Constants.rightNeoOneShooterMotorCanId, MotorType.kBrushless);
  private CANSparkMax rightMotorTwo = new CANSparkMax(Constants.rightNeoTwoShooterMotorCanId, MotorType.kBrushless);
  
  private Gains rightMotorGains = new Gains(0.15, 0.002, 5, 1023/20660.0, 300, 1.00);
  private Gains leftMotorGains = new Gains(0.15, 0.002, 5, 1023/20660.0, 300, 1.00);

  private SparkMaxPIDController leftMotorOnePidController = leftMotorOne.getPIDController();
  private SparkMaxPIDController leftMotorTwoPidController = leftMotorTwo.getPIDController();
  private SparkMaxPIDController rightMotorOnePidController = rightMotorOne.getPIDController();
  private SparkMaxPIDController rightMotorTwoPidController = rightMotorTwo.getPIDController();

  public NeoShooterSubsystem() {

    leftMotorOne.restoreFactoryDefaults();
    leftMotorTwo.restoreFactoryDefaults();
    rightMotorOne.restoreFactoryDefaults();
    rightMotorTwo.restoreFactoryDefaults();

    leftMotorOne.setIdleMode(IdleMode.kCoast);
    leftMotorTwo.setIdleMode(IdleMode.kCoast);
    rightMotorOne.setIdleMode(IdleMode.kCoast);
    rightMotorTwo.setIdleMode(IdleMode.kCoast);

    leftMotorOne.setInverted(Constants.leftNeoOneShooterMotorDefaultDirectionClockwise);
    leftMotorTwo.setInverted(Constants.leftNeoTwoShooterMotorDefaultDirectionClockwise);
    rightMotorOne.setInverted(Constants.rightNeoOneShooterMotorDefaultDirectionClockwise);
    rightMotorTwo.setInverted(Constants.rightNeoTwoShooterMotorDefaultDirectionClockwise);

    this.setGainsOnMotorPid(leftMotorOnePidController, leftMotorGains);
    this.setGainsOnMotorPid(leftMotorTwoPidController, leftMotorGains);
    this.setGainsOnMotorPid(rightMotorOnePidController, rightMotorGains);
    this.setGainsOnMotorPid(rightMotorTwoPidController, rightMotorGains);

    CommandScheduler.getInstance().registerSubsystem(this);
  }

  public void setShooterVelocityLeft(double motorOneRevolutionsPerMinute, double motorTwoRevolutionsPerMinute)
  {
    leftMotorOnePidController.setReference(motorOneRevolutionsPerMinute, CANSparkMax.ControlType.kVelocity);
    leftMotorTwoPidController.setReference(motorTwoRevolutionsPerMinute, CANSparkMax.ControlType.kVelocity);
  }

  public void setShooterVelocityRight(double motorOneRevolutionsPerMinute, double motorTwoRevolutionsPerMinute)
  {
    rightMotorOnePidController.setReference(motorOneRevolutionsPerMinute, CANSparkMax.ControlType.kVelocity);
    rightMotorTwoPidController.setReference(motorTwoRevolutionsPerMinute, CANSparkMax.ControlType.kVelocity);
  }

  public void setAllStop() {
    leftMotorOne.set(0.0);
    leftMotorTwo.set(0.0);
    rightMotorOne.set(0.0);
    rightMotorTwo.set(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  private void setGainsOnMotorPid(SparkMaxPIDController motorPid, Gains gains) {
    motorPid.setP(gains.kP);
    motorPid.setI(gains.kI);
    motorPid.setD(gains.kD);
    motorPid.setIZone(0.0);
    motorPid.setFF(0.000156);
    motorPid.setOutputRange(-1.0, 1.0);

    int smartMotionSlot = 0;
    motorPid.setSmartMotionMaxVelocity(Constants.neoMaximumRevolutionsPerMinute, smartMotionSlot);
    motorPid.setSmartMotionMinOutputVelocity(Constants.neoMaximumRevolutionsPerMinute/20, smartMotionSlot);
    motorPid.setSmartMotionMaxAccel(Constants.neoMaximumRevolutionsPerMinute/5, smartMotionSlot);
    motorPid.setSmartMotionAllowedClosedLoopError(Constants.neoMaximumRevolutionsPerMinute/100, smartMotionSlot);
  }

}
