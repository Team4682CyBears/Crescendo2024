// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class ToFSubsystem extends SubsystemBase {
  public static TimeOfFlight tofSensor = new TimeOfFlight(Constants.canID);

  public void RangeSubysytem(){
    tofSensor.setRangingMode(RangingMode.Medium, 140);
  }
  

  public boolean getRange(){
      double range = Units.metersToInches(tofSensor.getRange()/1000);
      if(range == 19.0){
        return false;
      }else if(range < 19.0){
        return true;
      }
      return false;
  }
//flashes the sensor
  public void blinkSensor(){
    tofSensor.identifySensor();
  }
//Standard deviation of distance measurment in millimeters
  public static final double getRangeSigma(){
    return  tofSensor.getRangeSigma();
  }
  //Returns true if sensor correctly measured distance from object
  public boolean isRangeValid(){
    return tofSensor.isRangeValid();
  }
  
}