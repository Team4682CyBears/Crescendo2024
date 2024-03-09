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
import frc.robot.common.FeederMode;
import frc.robot.common.NoteTofSensor;
import frc.robot.control.Constants;
import frc.robot.control.InstalledHardware;

/**
 * Forms a class for the feeder subsystem consisting of
 * a Bag motor and two TOF sensors
 */
public class FeederSubsystem extends SubsystemBase {
  // Bag motor
  private TalonSRX feederMotor = new TalonSRX(Constants.feederMotorCanId);
  // BeamBreak sensors
  private NoteTofSensor firstShooterBeambreakSensor = null;
  private NoteTofSensor secondShooterBeamBreakSensor = null;
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
    if (InstalledHardware.firstFeederToShooterTofInstalled){
      firstShooterBeambreakSensor = new NoteTofSensor(Constants.firstFeederToShooterTofCanId);
      firstShooterBeambreakSensor.setDisplayName("First Shooter TOF");
    }
    if (InstalledHardware.secondFeederToShooterTofInstalled){
      secondShooterBeamBreakSensor = new NoteTofSensor(Constants.secondFeederToShooterTofCanId);
      secondShooterBeamBreakSensor.setDisplayName("Second Shooter TOF");
    }
  }

  /**
   * A method to return the feeder mode
   * @return 
   */
  public FeederMode getFeederMode() {
    return feederMode;
  }

  /**
   * A method to detect the presence of a note at the dunker
   * @return true if the note is detected
   */
  public boolean isDunkerNoteDetected(){
    if (secondShooterBeamBreakSensor != null){
      return secondShooterBeamBreakSensor.isNoteDetected();
    }
    return false;
  }
  
  /**
   * A method to detect the presence of a note at the shooter
   * @return true if the note is detected
   */
  public boolean isShooterNoteDetected(){
    if (firstShooterBeambreakSensor != null){
      return firstShooterBeambreakSensor.isNoteDetected();
    }
    return false;
  }

  /**
   * periodic for this subsystem. Called once per scheduler run (20ms) 
   */
  @Override
  public void periodic() {
    if(this.secondShooterBeamBreakSensor != null) {
      this.secondShooterBeamBreakSensor.publishTelemetery();
    }
    if(this.firstShooterBeambreakSensor != null) {
      this.firstShooterBeambreakSensor.publishTelemetery();
    }
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
   * A method to set the motor to the given percent in the 
   * direction of the shooter or feeder depending on the mode
   * @param percent (between 0 and 1)
   */
  public void setFeederSpeed(double percent) {
    int direction = feederMode == FeederMode.FeedToShooter ? shooterDirection : feederDirection; 
    feederMotor.set(TalonSRXControlMode.PercentOutput, percent * direction);
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