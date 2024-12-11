// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: FeederSubsystem.java
// Intent: Forms the prelminary code for feeder subsystem.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

/**
 * Forms a class for the feeder subsystem consisting of a Bag motor
 */
public class FeederSubsystem extends SubsystemBase {
  // Bag motor
  private TalonSRX feederMotor = new TalonSRX(Constants.FEEDER_MOTOR_CAN_ID);

  /**
   * Constructor for the IntakeSubsystem
   */
  public FeederSubsystem() {
    feederMotor.setNeutralMode(NeutralMode.Brake);
    TalonSRXConfiguration config = new TalonSRXConfiguration();
    config.peakCurrentLimit = 40; // the peak current, in amps
    config.peakCurrentDuration = 1500; // the time at the peak current before the limit triggers, in ms
    config.continuousCurrentLimit = 30; // the current to maintain if the peak limit is triggered
    feederMotor.configAllSettings(config); // apply the config settings; this selects the quadrature encoder
  }


  /**
   * A method to set the motor to the given percent in the 
   * direction of the shooter or feeder depending on the mode
   * @param percent (between 0 and 1)
   */
  public void setForwardSpeed() {
    feederMotor.set(TalonSRXControlMode.PercentOutput, Constants.FORWARD_SPEED);
  }

  public void setBackwardSpeed() {
    feederMotor.set(TalonSRXControlMode.PercentOutput, Constants.BACKWARD_SPEED);
  }

  /**
   * A method to stop the intake subsystem
   */
  public void setAllStop() {
    feederMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
  }

  /**
   * This method will be called once per scheduler run during simulation
   */
  @Override
  public void simulationPeriodic() {
  }
}
