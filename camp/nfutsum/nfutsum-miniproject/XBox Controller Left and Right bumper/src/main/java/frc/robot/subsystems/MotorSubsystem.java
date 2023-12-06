// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: MotorSubsystem.java
// Intent: Creates a subsystem for the motor
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 
package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


/** 
 * Class that creates Motor subsystem 
*/
public class MotorSubsystem extends SubsystemBase {

  private WPI_TalonFX motor = new WPI_TalonFX(Constants.motorCanId);
  
  /** 
   * Default constructor
   */
  public MotorSubsystem() {
    motor.configFactoryDefault();
    motor.setInverted(false);
    motor.set(ControlMode.PercentOutput, 0.0);
  }

  /**
   * Method that sets forward speed
   *
   * @return forward command
   */
   public void setForward()
  {
    motor.set(Constants.speed);
  } 

/**
   * Method that sets bacxkward speed
   *
   * @return backward command
   */
  public void setBackward()
  {
    motor.set(Constants.reverseValue*Constants.speed);
  }

  /**
   * Method that sets motor speed to 0
   *
   * @return stop command
   */
  public void setStop()
  {
    motor.set(0.0);
  }
}
