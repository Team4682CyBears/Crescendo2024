package frc.robot.common;

public class MotorUtils
{

    /**
    * a method to validate speed input and throw if the speed value is invalid   
    *
    * @param  speed - target speed of the motor
    * @param  prependMessage - when speed is invalid, a string added to the front of the exception message
    * @param  appendMessage - when speed is invalid, a string added to the end of the exception message
    */
    public static void validateMotorSpeedInput(
        double speed,
        String prependMessage,
        String appendMessage)
    {
        if(speed > 1.0 || speed < -1.0)
        {
            throw new IllegalArgumentException(
                prependMessage == null ? "" : prependMessage +
                "input outside of acceptable motor speed range (valid range from -1.0 to 1.0)" +
                appendMessage == null ? "" : appendMessage);
        }        
    }
    
    /**
    * A method to make sure that values are retained within the boundaries 
    *
    * @param  value - target value 
    * @param  minBoundary - when value is below the minimum boundary the minBoundary is used as the returned value
    * @param  maxBoundary - when value is above the maximum boundary the maxBoundary is used as the returned value
    * @return the target or trimmed value
    */
    public static double truncateValue(
        double value,
        double minBoundary,
        double maxBoundary)
    {
        double trimmedValue = value;
        if(value < minBoundary)
        {
            trimmedValue = minBoundary;
        }
        else if (value > maxBoundary)
        {
            trimmedValue = maxBoundary;
        }
        return trimmedValue;
    }  
}