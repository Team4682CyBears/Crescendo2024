// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2024
// File: ToFSubsystem.java
// Intent: Subsystem for ToF sensor to detect when note enters its range
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 
package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ToFSubsystem extends SubsystemBase {
  public static TimeOfFlight tofSensor = new TimeOfFlight(Constants.canID);

  /*
   * Sets ranging mode to medium
   */
  public void RangeSubysytem(){
    tofSensor.setRangingMode(RangingMode.Medium, 140);
  }
  
/*
 * Gets the range of the sensor and returns true if its less than 19 inches, meaning a note is detected
 */
  public boolean getRange(){
      double range = Units.metersToInches(tofSensor.getRange()/1000);
      if(range == 19.0){
        return false;
      }else if(range < 19.0){
        return true;
      }
      return false;
  }
/*
 * flashes the sensor
*/
  public void blinkSensor(){
    tofSensor.identifySensor();
  }
/*
 * Returns standard deviation of distance measurment in millimeters
 */
  public static final double getRangeSigma(){
    return  tofSensor.getRangeSigma();
  }
  /*
   * Returns true if sensor correctly measured distance from object
   */
  public boolean isRangeValid(){
    return tofSensor.isRangeValid();
  }
  
}