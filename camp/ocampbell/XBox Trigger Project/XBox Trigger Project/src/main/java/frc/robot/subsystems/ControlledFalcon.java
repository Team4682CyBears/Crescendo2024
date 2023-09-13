// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;


public class ControlledFalcon extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

    TalonFX motor = new TalonFX(Constants.portControleldFalcon);


    TalonFXConfiguration config = new TalonFXConfiguration();


  public ControlledFalcon() {
    //this.intitalizeControlledFalconState();
    //CommandScheduler.getInstance().registerSubsystem(this);

    config.supplyCurrLimit.enable = true;
    config.supplyCurrLimit.triggerThresholdCurrent = 40; // the peak supply current, in amps
    config.supplyCurrLimit.triggerThresholdTime = 1.5; // the time at the peak supply current before the limit triggers, in sec
    config.supplyCurrLimit.currentLimit = 30; // the current to maintain if the peak supply limit is triggered
    motor.configAllSettings(config); // apply the config settings; this selects the quadrature encoder
  }


  public void setFalconRelitiveSpeed(double controlledFalconSpeed) {
    motor.set(TalonFXControlMode.PercentOutput, controlledFalconSpeed);
    System.out.print(controlledFalconSpeed);
    System.out.print("test");
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  private void intitalizeControlledFalconState() {

  }



}
