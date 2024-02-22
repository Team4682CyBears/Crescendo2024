// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2024
// File: TofSubsystem.java
// Intent: Subsystem for ToF sensor to detect when note enters its range
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 
package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


/**
 * Forms a class for the TofSubsystem that detects when a note is present. 
 */
public class TofSubsystem extends SubsystemBase {
  private static TimeOfFlight tofSensor;
  private int canID;
  private double currentRangeInches;
  private boolean currentRangeIsValid;

  public TofSubsystem(int canID){
    tofSensor = new TimeOfFlight(canID);
    this.canID = canID;
    // short mode is accurate to 1.3m 
    // 20ms sample time matches robot update rate
    tofSensor.setRangingMode(RangingMode.Short, 20);
    System.out.println("==== DONE CONFIG of TOF SENSOR at CanID " + canID);
  }
  
  /**
   * A method to flash the sensor
   */
  public void blinkSensor(){
    tofSensor.identifySensor();
  }

  /**
   * A method to detect the presence of a note
   * @return true if note is detected
   */
  public boolean isNoteDetected(){
    double noteDetectedThreshold = 8.0;
    if(currentRangeIsValid && (currentRangeInches < noteDetectedThreshold)){
      return true;
    }
    return false;
  }

  /**
   * A method to get the sensor range in inches
   * @return
   */
  public double getRangeInches(){
      return currentRangeInches;
  }

  /**
   * A method to return the standard deviation of the measurement
   * @return standard deviation in millimeters
   */
  public final double getRangeSigma(){
    return  tofSensor.getRangeSigma();
  }

  /**
   * A method to return the sensor status
   * @return true if the sensor correctly measured the distance
   */
  public boolean isRangeValid(){
    return tofSensor.isRangeValid();
  }

  /**
   * A method to read the sesnor value
   */
  private void readSensor(){
    currentRangeInches = Units.metersToInches(tofSensor.getRange()/1000);
    currentRangeIsValid = isRangeValid();
  }

  /**
   * periodic for this subsystem. Called once per scheduler run (20ms) 
   */
  @Override
  public void periodic(){
    readSensor();
    SmartDashboard.putNumber("TOF ID " + canID + " Range Inches" , currentRangeInches);
    SmartDashboard.putBoolean("TOF ID " + canID + " Note Detected", isNoteDetected());
  } 
}