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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class TofSubsystem extends SubsystemBase {
  private static TimeOfFlight tofSensor;
  private double currentRangeInches;

  public TofSubsystem(int canID){
    tofSensor = new TimeOfFlight(canID);
    // short mode is accurate to 1.3m 
    // 20ms sample time matches robot update rate
    tofSensor.setRangingMode(RangingMode.Short, 20);
    CommandScheduler.getInstance().registerSubsystem(this);
    System.out.println("==== DONE CONFIG of TOF SENSOR");
    System.out.println("Range = " + getRangeInches());
  }
  
  //flashes the sensor
  public void blinkSensor(){
    tofSensor.identifySensor();
  }

  // detect presence of a note
  public boolean isNoteDetected(){
    double notDetectedThreshold = 8.0;
    if(currentRangeInches >= notDetectedThreshold){
      return false;
    }else if(currentRangeInches < notDetectedThreshold){
      return true;
    }
    return false;
  }

  public double getRangeInches(){
      return Units.metersToInches(tofSensor.getRange()/1000);
  }

  //Standard deviation of distance measurment in millimeters
  public static final double getRangeSigma(){
    return  tofSensor.getRangeSigma();
  }

  //Returns true if sensor correctly measured distance from object
  public boolean isRangeValid(){
    return tofSensor.isRangeValid();
  }

  // periodic for this subsystem
  @Override
  public void periodic(){
    this.currentRangeInches = getRangeInches();
    SmartDashboard.putNumber("TOF Range Inches", currentRangeInches);
    SmartDashboard.putBoolean("Note Detected", isNoteDetected());
  }
  
}