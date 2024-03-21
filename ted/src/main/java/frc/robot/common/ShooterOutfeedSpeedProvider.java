package frc.robot.common;

import java.security.InvalidParameterException;

import frc.robot.control.Constants;
import frc.robot.subsystems.ShooterAngleSubsystem;

/**
 * A singleton class to provide the speed for spin-up and for 
 */
public class ShooterOutfeedSpeedProvider {

    private static ShooterOutfeedSpeedProvider currentSingleton = null;

    private final double spinUpSpeedMulitiplier = 0.8; // spin-up at 80% of target speed
    private final double angleStandardOffset = 4;
    private final double minmumAngleBoundary = 0;
    private final double speakerToAmpAngleBoundary = 90; 
    private final double maximumAngleBoundary = 180.001; // just so nothing fails at 180

    // todo - come up with better alternative for shot speeds using a physics model
    // for now keep it simple and just use a lookup table with pre-set values instead
    // important to order these ascending in angle as interpolation can be broken otherwise
    private final double[][] outfeedSpeedTable = {
        {this.minmumAngleBoundary, Constants.shooterOutfeedSpeedForAngleShootFromSourceWing},
        {Constants.shooterAngleMinDegrees - this.angleStandardOffset, Constants.shooterOutfeedSpeedForAngleShootFromSourceWing},
        {Constants.shooterAngleMinDegrees, Constants.shooterOutfeedSpeedForAngleShootFromSourceWing},
        {Constants.shooterAngleShootFromSourceWing, Constants.shooterOutfeedSpeedForAngleShootFromSourceWing},
        {Constants.shooterAngleShootFromStage, Constants.shooterOutfeedSpeedForAngleShootFromStage},
        {Constants.shooterAngleShootFromNote, Constants.shooterOutfeedSpeedForAngleShootFromNote},
        {Constants.shooterAngleShootFromSpeaker, Constants.shooterOutfeedSpeedForAngleShootFromSpeaker},
        {Constants.shooterAngleShootFromSpeaker + this.angleStandardOffset, Constants.shooterOutfeedSpeedForAngleShootFromSpeaker},
        {this.speakerToAmpAngleBoundary, Constants.shooterOutfeedSpeedForAngleShootFromSpeaker}, 
        {Constants.shooterAngleShootFromAmp - this.angleStandardOffset, Constants.shooterOutfeedSpeedForAngleShootFromAmp},
        {Constants.shooterAngleShootFromAmp, Constants.shooterOutfeedSpeedForAngleShootFromAmp},
        {this.maximumAngleBoundary - this.angleStandardOffset, Constants.shooterOutfeedSpeedForAngleShootFromAmp},
        {this.maximumAngleBoundary, Constants.shooterOutfeedSpeedForAngleShootFromAmp},
    };

    private ShooterAngleSubsystem angleSubsystem = null;

    /**
     * Private constructor to provide singleton pattern
     * @param currentAngleSubsystem - the existing and valid angle subsystem
     */
    private ShooterOutfeedSpeedProvider(ShooterAngleSubsystem currentAngleSubsystem) {
        this.angleSubsystem = currentAngleSubsystem;
    }

    // STATIC Methods

    /**
     * The instance getter to provide the single ShooterOutfeedSpeedProvider for the 
     * robot program running in memory.
     * @param currentAngleSubsystem - a valid and already initialized angle subsystem
     * @return - the single ShooterOutfeedSpeedProvider for the robot program running in memory
     */
    public static ShooterOutfeedSpeedProvider getInstance(ShooterAngleSubsystem currentAngleSubsystem) {
        if(ShooterOutfeedSpeedProvider.currentSingleton == null) {
            // specifically call something simple on the subsystem 
            // this will confirm it properly initialized prior to creation
            double value = currentAngleSubsystem.getAngleDegrees();
            currentSingleton = new ShooterOutfeedSpeedProvider(currentAngleSubsystem);
        }
        return currentSingleton;

    }

    // PUBLIC Methods

    /**
     * A method to obtain the appropriate shot speed for the current angle of the shooter
     * @param targetAngle - the angle to obtain the target speed for
     * @return a double representing the motor output shaft absolute rotational speed in revolutions per minute (RPM)
     */
    public double getShotSpeedForAngle(double targetAngle) {
        return this.getShotSpeedFromLookupTable(targetAngle);
    }

    /**
     * A method to obtain the appropriate spin up speed for the current angle of the shooter
     * @param targetAngle - the angle to obtain the target speed for
     * @return a double representing the motor output shaft absolute rotational speed in revolutions per minute (RPM)
     */
    public double getSpinUpSpeedForAngle(double targetAngle) {
        return this.getShotSpeedForAngle(targetAngle) * this.spinUpSpeedMulitiplier;
    }

    /**
     * A method to obtain the appropriate shot speed for the current angle of the shooter
     * @return a double representing the motor output shaft absolute rotational speed in revolutions per minute (RPM)
     */
    public double getShotSpeedForCurrentAngle() {
        return this.getShotSpeedForAngle(this.angleSubsystem.getAngleDegrees());
    }

    /**
     * A method to obtain the appropriate spin up speed for the current angle of the shooter
     * @return a double representing the motor output shaft absolute rotational speed in revolutions per minute (RPM)
     */
    public double getSpinUpSpeedForCurrentAngle() {
        return this.getSpinUpSpeedForAngle(this.angleSubsystem.getAngleDegrees());
    }
    
    // PRIVATE Methods

    /**
     * A method that will use the pre-configured lookup table to produce a speed that is
     * linear interpoloated based on target angle
     * @param targetAngle
     * @return the target speed of the shooter
     */
    private double getShotSpeedFromLookupTable(double targetAngle) {

        double lastAngle = 0;
        double nextAngle = 0;      
        double lastSpeed = 0;
        double nextSpeed = 0;

        // see if the lookup table contains entries that enclose the target angle
        int i = 0;
        boolean rangeFound = false;
        while(!rangeFound) {
            if(outfeedSpeedTable.length > i) {
                lastAngle = nextAngle;
                lastSpeed = nextSpeed;
                nextAngle = outfeedSpeedTable[i][0];
                nextSpeed = outfeedSpeedTable[i][1];
                if(targetAngle >= lastAngle && targetAngle < nextAngle) {
                    rangeFound = true;
                    break;
                }
            }
            else {
                break;
            }
            ++i;
        }

        if(!rangeFound) {
            throw new InvalidParameterException("The provided target angle is not within proper bounds!");
        }
        else{
            // do interpolation on speed to get shot speed
            double numerator = (lastSpeed * (nextAngle - targetAngle)) + (nextSpeed * (targetAngle - lastAngle));
            double denominator = (nextAngle - lastAngle); 
            return (numerator/denominator);
        }
    }
}
