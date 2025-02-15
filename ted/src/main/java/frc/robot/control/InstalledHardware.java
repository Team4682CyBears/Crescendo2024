// ***********************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: InstalledHardware.java
// Intent: Forms a listing of switches that will help to debug code better as hardware is available (or not available).
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

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
    
    // Onboard Hardware - Orentation/Navigation Hardware
    public static final boolean wifiRadioInstalled = true;
    public static final boolean navx2Installed = false;
    public static final boolean navx1Installed = true;
    public static final boolean navxInstalled = navx1Installed || navx2Installed;
    public static final boolean limelightInstalled = true;

    // External Input Hardware
    public static final boolean driverXboxControllerInstalled = true;
    public static final boolean coDriverXboxControllerInstalled = true;

    // DriveTrain Related Hardware
    public static final boolean tedDrivetrainInstalled = true; // true is ted, false is babybear/minibear
    public static final boolean leftFrontDriveInstalled = true;
    public static final boolean leftRearDriveInstalled = true;
    public static final boolean rightFrontDriveInstalled = true;
    public static final boolean rightRearDriveInstalled = true;

    // Intake Related Hardware
    public static final boolean intakeInstalled = true;

    // Feeder Related Hardware
    public static final boolean feederInstalled = true;

    // TOF Sensor Hardware
    // Important! You must disable any TOF sensor that is not installed!! 
    // If you try to configure a TOF sensor that is not installed
    // the other TOF sensors that are installed will not work. 
    public static final boolean intakeTofInstalled = true;
    public static final boolean firstFeederToShooterTofInstalled = true;
    public static final boolean secondFeederToShooterTofInstalled = true;
    public static final boolean feederToDunkerTofInstalled = false;

    // Shooter Related Hardware
    public static final boolean shooterOutfeedInstalled = true;
    public static final boolean shooterAngleInstalled = true;
    // for testing, to decrease the power of the shooter angle mechanism, 
    // reduce the left motor gear box to 10x (instaed of 100x)
    // and disconnect the right motor from the chain. 
    public static final boolean shooterRightAngleMotorrInstalled = true;
    public static final boolean shooterAngleCanCoderInstalled = true; 
    // for testing shooter angle via the shuffleboard
    // ensure this is DISABLED for competitions, as it can sometimes crash shuffleboard
    public static final boolean setShooterAngleFromShuffleboard = false;

    // Climber Sensor Related Hardware
    public static final boolean leftClimberInstalled = true;
    public static final boolean rightClimberInstalled = true;
    public static final boolean leftClimberSensorInstalled = true;
    public static final boolean rightClimberSensorInstalled = true;

    //LED Hardware
    public static final boolean LEDSInstalled = true;
}
