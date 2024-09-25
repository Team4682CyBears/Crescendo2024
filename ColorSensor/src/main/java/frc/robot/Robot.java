// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2024
// File: Robot.java
// Intent: Shows true and false on shuffleboard when note is detected 
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 
package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;

import com.revrobotics.ColorSensorV3;
import com.revrobotics.ColorMatchResult;

import com.revrobotics.ColorMatch;

public class Robot extends TimedRobot {
  
  private final I2C.Port i2cPort = I2C.Port.kOnboard;

  //A Rev Color Sensor V3 object is constructed with an I2C port as a parameter.
  private final ColorSensorV3 m_colorSensor = new ColorSensorV3(i2cPort);

  //Used to register and detect known colors.
  private final ColorMatch m_colorMatcher = new ColorMatch();

  //Note color value (orange). Blue background will be used to 
  private final Color blue = new Color(0.143, 0.427, 0.429);
  private final Color noteColor = new Color(0.558, 0.365, 0.077);
  
  @Override
  public void robotInit() {
    m_colorMatcher.addColorMatch(noteColor); 
    m_colorMatcher.addColorMatch(blue);
  }

  @Override
  public void robotPeriodic() {
    //Get the most likely color. Works best when within 2 inches and perpendicular to surface of interest.
    Color detectedColor = m_colorSensor.getColor();
    
    //Runs the color match algorithm on our detected color
    String colorString;
    ColorMatchResult match = m_colorMatcher.matchClosestColor(detectedColor);

    boolean isNoteColor = false;

     if (match.color == noteColor) {
      colorString = "Note";
      isNoteColor = true;
    }else {
      colorString = "Unknown"; 
    }
    
    //Displays green(true) or red(false) on shuffleboard on whether note is detected or not.
    SmartDashboard.putBoolean("Note Detected", isNoteColor);
  }
}

