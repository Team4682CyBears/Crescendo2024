// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


/** Creates Motor subsystem */
public class Motor extends SubsystemBase {

  private WPI_TalonFX Motor = new WPI_TalonFX(Constants.motorCanId);
    
  public Motor() {
    }

  /**
   * Method that sets forward speed
   *
   * @return forward command
   */
   public void forward()
  {
    Motor.set(Constants.speed);
  } 

/**
   * Method that sets bacxkward speed
   *
   * @return backward command
   */
  public void backward()
  {
    Motor.set(-1*Constants.speed);
  }

  /**
   * Method that sets motor speed to 0
   *
   * @return stop command
   */
  public void stop()
  {
    Motor.set(0.0);
  }
}
