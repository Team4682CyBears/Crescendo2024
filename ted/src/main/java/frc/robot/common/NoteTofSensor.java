
// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2024
// File: NoteTofSensor.java
// Intent: Subsystem for ToF sensor to detect when note enters its range
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 
package frc.robot.common;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;

/**
 * Forms a class for the TofSubsystem that detects when a note is present. 
 */
public class NoteTofSensor implements Sendable{

  private static final double noteDetectedThreshold = 8.0;
  private TimeOfFlight tofSensor;
  private int canID;
  private String displayName;

  public NoteTofSensor(int canID){
    tofSensor = new TimeOfFlight(canID);
    this.canID = canID;
    this.displayName = "TOF ID " + this.canID;
    // short mode is accurate to 1.3m 
    // 20ms sample time matches robot update rate
    tofSensor.setRangingMode(RangingMode.Short, 20);
    System.out.printf("==== DONE CONFIG of TOF SENSOR at CanID %d \n", canID);
  }
  
  /**
   * A method to flash the sensor
   */
  public void blinkSensor(){
    tofSensor.identifySensor();
  }

    /** 
   * A method to return the display name
   * @return - the display name
   */
  public String getDisplayName(){
    return displayName;
  }

  /**
   * A method to get the sensor range in inches
   * @return - the current range in inches
   */
  public double getRangeInches(){
      return Units.metersToInches(tofSensor.getRange()/1000);
  }

  /**
   * A method to return the standard deviation of the measurement
   * @return standard deviation in millimeters
   */
  public final double getRangeSigma(){
      return tofSensor.getRangeSigma();
  }

  /**
   * A method to detect the presence of a note
   * @return true if note is detected
   */
  public boolean isNoteDetected(){
    double currentRangeInches = this.getRangeInches();
    if(this.isRangeValid() && (currentRangeInches < noteDetectedThreshold)){
      return true;
    }
    return false;
  }

  /**
   * A method to return the sensor status
   * @return true if the sensor correctly measured the distance
   */
  public boolean isRangeValid(){
      return tofSensor.isRangeValid();
  }

  /**
   * A method that will publish the telemetry associated with this TOF sensor to Shuffleboard
   */
  @Override
  public void initSendable(SendableBuilder builder) {
    builder.addDoubleProperty(displayName + " Range Inches" , this::getRangeInches, null);
    builder.addBooleanProperty(displayName + " Note Detected", this::isNoteDetected, null);
    builder.addBooleanProperty(displayName + " Range Is Valid", this::isRangeValid, null);
    builder.addStringProperty(displayName + " TOF Status", () -> this.tofSensor.getStatus().toString(), null);
  } 

  /**
   * Updates the display name of this sensor
   * @param displayName - the updated name to associate to this sensor
   */
  public void setDisplayName(String displayName){
      this.displayName = displayName;
  }

}