// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: Constants.java
// Intent: Forms key constants required for this robot.
// ************************************************************

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */

package frc.robot.control;

public class Constants {
    // *****************************************************************
    // input device constants
    public static final int portDriverController = 0;

    // ******************************************************************
    // intake will run until note is detected or this timeout has expired
    public static final double intakeTimeoutSeconds = 4.0;

    // ******************************************************************
    // feeder constants
    public static final int feederMotorCanId = 15;
    public static final int firstFeederToShooterTofCanId = 16;
    public static final int secondFeederToShooterTofCanId = 17;
    // feederSpeed is [0.0 .. 1.0]
    // it runs in one direction for the shooter 
    // and the opposite direction for the dunker/amp
    public static final double feederSpeed = 0.50;
    public static final double feederReverseSpeed = 0.10;
    // feeder will run until note is detected or this timeout has expired
    public static final double feederTimeoutSeconds = 3.0;
    public static final double feederLaunchTimeoutSecondsInAuto = .75;
    public static final double feederLaunchTimeoutSecondsInTele = 2.00;
    public static final double feederRewindSeconds = intakeTimeoutSeconds;

    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
    }
    
}
