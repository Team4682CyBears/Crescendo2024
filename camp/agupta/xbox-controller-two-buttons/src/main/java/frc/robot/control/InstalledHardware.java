// ***********************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: InstalledHardware.java
// Intent: Forms a listing of switches that will help to debug code better as hardware is available (or not available).
// ************************************************************

package frc.robot.control;

/**
 * A class devoted to installed hardware constants.  Use this to decide if hardware is enabled on the robot or not.
 * All developers to use this to protect their subsystems and commands from using hardware that is not actually installed
 * on the robot at hand.  Used to assist in development stages and make it easier to quickly remove a piece of hardware
 * from the robot.
 */
public class InstalledHardware 
{
    // Installed I/O Electronic hardware
    // Hardware that is N/A - mostly because these pieces of hardware have no communication with the RoboRio
    // batteryInstalled = true;
    // voltageRegulartorModuleInstalled = true;
    // robotEnabledLightInstalled = true;
    // roboRioInstalled = true; - if it wasn't installed then the code wouldn't be running

    // Basic Hardware
    public static final boolean powerDistributionPanelInstalled = true; 

    // External Input Hardware
    public static final boolean driverXboxControllerInstalled = true;

    // ** -- bag motors on TED robot
    // Bag Related Hardware
    public static final boolean bagInstalled = true;

    // ************************************************************
    //
    // UNUSED HARDWARE
    //
    // Onboard Hardware - Orentation/Navigation Hardware
    public static final boolean wifiRadioInstalled = false;
    public static final boolean navx2Installed = false;
    public static final boolean navx1Installed = false;
    public static final boolean navxInstalled = navx1Installed || navx2Installed;
    public static final boolean limelightInstalled = false;

    // External Input Hardware
    public static final boolean coDriverXboxControllerInstalled = false;

    // DriveTrain Related Hardware
    public static final boolean tedDrivetrainInstalled = false; // true is ted, false is babybear/minibear
    public static final boolean leftFrontDriveInstalled = false;
    public static final boolean leftRearDriveInstalled = false;
    public static final boolean rightFrontDriveInstalled = false;
    public static final boolean rightRearDriveInstalled = false;

    // Intake Related Hardware
    public static final boolean intakeInstalled = false;

    // TOF Sensor Hardware
    // Important! You must disable any TOF sensor that is not installed!! 
    // If you try to configure a TOF sensor that is not installed
    // the other TOF sensors that are installed will not work. 
    public static final boolean intakeTofInstalled = false;
    public static final boolean firstBagToShooterTofInstalled = false;
    public static final boolean secondBagToShooterTofInstalled = false;
    public static final boolean bagToDunkerTofInstalled = false;

    // Shooter Related Hardware
    public static final boolean shooterOutfeedInstalled = false;
    public static final boolean shooterAngleInstalled = false;
    // for testing, to decrease the power of the shooter angle mechanism, 
    // reduce the left motor gear box to 10x (instaed of 100x)
    // and disconnect the right motor from the chain. 
    public static final boolean shooterRightAngleMotorrInstalled = false;
    public static final boolean shooterAngleCanCoderInstalled = false; 
    // for testing shooter angle via the shuffleboard
    // ensure this is DISABLED for competitions, as it can sometimes crash shuffleboard
    public static final boolean setShooterAngleFromShuffleboard = false;

    // Climber Sensor Related Hardware
    public static final boolean leftClimberInstalled = false;
    public static final boolean rightClimberInstalled = false;
    public static final boolean leftClimberSensorInstalled = false;
    public static final boolean rightClimberSensorInstalled = false;
}
