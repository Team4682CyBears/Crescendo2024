// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: BagSubsystem.java
// Intent: Forms the prelminary code for bag subsystem.
// ************************************************************

// declare package containing class
package frc.robot.subsystems;

// import motor controller libraries
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonSRXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;

// import wpi libraries
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// import local classes
import frc.robot.control.Constants;
import frc.robot.common.BagMode;

/**
 * Forms a class for the bag subsystem
 */
public class BagSubsystem extends SubsystemBase {
  // create new instance of bag motor controller class
  private TalonSRX bagMotor = new TalonSRX(Constants.bagMotorCanId);
  // vendordeps library:
  // https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix5-frc2024-latest.json

  // Direction Mode default is forward
  BagMode bagMode = BagMode.Forward;
  private int backward = -1; // set 1 for not inverted, set -1 for inverted
  private int forward = 1; // set 1 for not inverted, set -1 for inverted

  /**
   * Constructor for the IntakeSubsystem
   */
  public BagSubsystem() {
    bagMotor.setNeutralMode(NeutralMode.Brake); // set motor off

    // set configuration
    // https://api.ctr-electronics.com/phoenix/release/java/com/ctre/phoenix/motorcontrol/can/TalonSRX.html
    TalonSRXConfiguration config = new TalonSRXConfiguration(); // create a new config instance
    config.peakCurrentLimit = 40; // the peak current, in amps
    config.peakCurrentDuration = 1500; // the time at the peak current before the limit triggers, in ms
    config.continuousCurrentLimit = 30; // the current to maintain if the peak limit is triggered
    bagMotor.configAllSettings(config); // apply the config settings; this selects the quadrature encoder
  }

  /**
   * A method to set the bag mode
   * @param bagMode
   */
  public void setBagMode(BagMode bagMode) {
    // If the bag mode is changing, stop the motor
    // this prevents the motor from going from +1 to -1 abruptly
    // If the bag mode is not changing, the motor can continute to run at the 
    // previous speed
    if (bagMode != this.bagMode){
      setAllStop();
    }
    this.bagMode = bagMode;
  }

  /**
   * A method to stop the intake subsystem
   */
  public void setAllStop() {
    bagMotor.set(TalonSRXControlMode.PercentOutput, 0.0);
  }

  /**
   * A method to set the motor to the given percent in the 
   * direction of the shooter or bag depending on the mode
   * @param percent (between 0 and 1)
   */
  public void setBagSpeed(double percent) {

    // change bag direction based of bag mode shooter or bag
    int direction = bagMode == BagMode.Forward ? forward : backward; 

    // set bag speed
    bagMotor.set(TalonSRXControlMode.PercentOutput, percent * direction);
  }



    
}


