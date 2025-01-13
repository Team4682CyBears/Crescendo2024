// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: FeederSubsystem.java
// Intent: Forms the prelminary code for feeder subsystem.
// ************************************************************

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.control.Constants;
import frc.robot.control.InstalledHardware;
import frc.robot.common.FeederMode;

/**
 * Forms a class for the feeder subsystem consisting of
 * a Bag motor and two TOF sensors
 */
public class FeederSubsystem extends SubsystemBase {
  // Bag motor
  private TalonSRX feederMotor = new TalonSRX(Constants.feederMotorCanId);
  // vendordeps library:
  // https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix5-frc2024-latest.json

  // Direction Mode default is feed to shooter
  FeederMode feederMode = FeederMode.FeedToShooter;
  private int shooterDirection = -1; // set 1 for not inverted, set -1 for inverted
  private int feederDirection = 1; // set 1 for not inverted, set -1 for inverted

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
   * A method to set the feeder mode
   * @param feederMode
   */
  public void setFeederMode(FeederMode feederMode) {
    // If the feeder mode is changing, stop the motor
    // this prevents the motor from going from +1 to -1 abruptly
    // If the feeder mode is not changing, the motor can continute to run at the 
    // previous speed
    if (feederMode != this.feederMode){
      setAllStop();
    }
    this.feederMode = feederMode;
  }

  /**
   * A method to stop the intake subsystem
   */
  public void setAllStop() {
    feederMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
  }

  /**
   * A method to set the motor to the given percent in the 
   * direction of the shooter or feeder depending on the mode
   * @param percent (between 0 and 1)
   */
  public void setFeederSpeed(double percent) {

    // change feeder direction based of feeder mode shooter or feeder
    // if in wrong direction, don't shoot
    int direction = feederMode == FeederMode.FeedToShooter ? shooterDirection : feederDirection; 

    // set feeder speed
    feederMotor.set(TalonSRXControlMode.PercentOutput, percent * direction);
  }



    
}


