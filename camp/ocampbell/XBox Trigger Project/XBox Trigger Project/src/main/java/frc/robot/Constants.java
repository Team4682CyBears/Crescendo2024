// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// File: Constants.java
// Intent: Forms a class that holds common variables.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  
      // *****************************************************************
    // standard stuff constants - motors rotation, etc.
    public static final double DegreesPerRevolution = 360.0;
    // NEO maximum RPM 
    public static final double neoMaximumRevolutionsPerMinute = 5676;
    // NEO 550 maximum RPM - see: https://www.revrobotics.com/rev-21-1651/#:~:text=The%20following%20specifications%20for%20the%20NEO%20550%20Brushless,Motor%20Kv%3A%20917%20Kv%20Free%20Speed%3A%2011000%20RPM
    public static final double neoFiveFiveZeroMaximumRevolutionsPerMinute = 11000;
    // this uses the halls effect sensor when plugged into the spark max
    // see: https://www.revrobotics.com/rev-21-1650/ where it says "42 counts per rev."
    public static final double RevNeoEncoderTicksPerRevolution = 42;

    // *****************************************************************


    // input device constants
    public static final int portDriverController = 0;
    public static final int portCoDriverController = 1;


    // motor can ports
    public static final int portControleldFalcon = 1;

  
  
    public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
