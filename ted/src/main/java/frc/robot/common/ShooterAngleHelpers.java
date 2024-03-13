// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: ShooterAngleHelper.java
// Intent: Forms a class of helper functions for shooter angle
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.common;

import frc.robot.control.Constants;

/**
 * Forms a class that will help with computing the angle of the shooter when using vision
 */
public class ShooterAngleHelpers {
    // coefficients for a piece-wise linear function composed of two linear equations
    // and a breakpoint where they meet
    private static double farSlope = -4.65;
    private static double farOffset = 51.6;
    private static double closeSlope = -17.9;
    private static double closeOffset = 79.1;
    private static double nearFarBreakpoint = 2.0;

    /**
     * Method that will return what angle the shooter should be at to score a note given a distance
     * @param distance distance from speaker in meters
     * @return
     */
    public static double shooterAngleFromDistance(double distance){
        double slope = 0.0;
        double offset = 0.0;
        if(distance < nearFarBreakpoint){
            slope = closeSlope;
            offset = closeOffset;
        }
        else{
            slope = farSlope;
            offset = farOffset;
        }
        double angle = slope * distance + offset;
        angle = MotorUtils.clamp(angle, Constants.shooterAngleMinDegrees, Constants.shooterAngleMaxDegrees);
        return angle;
    }

}
