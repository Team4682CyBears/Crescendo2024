// ***********************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
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
    public static final boolean limelightInstalled = false;

    // External Input Hardware
    public static final boolean driverXboxControllerInstalled = true;
    public static final boolean coDriverXboxControllerInstalled = true;

    // DriveTrain Related Hardware
    public static final boolean leftFrontDriveInstalled = true;
    public static final boolean leftRearDriveInstalled = true;
    public static final boolean rightFrontDriveInstalled = true;
    public static final boolean rightRearDriveInstalled = true;

    // Controller button install control
    public static final boolean applyBasicDriveToPointButtonsToDriverXboxController = false;
    public static final boolean applyDriveTrajectoryButtonsToDriverXboxController = false;

    // Arm Related Hardware
    public static final boolean horizontalArmMotorInstalled = true;
    public static final boolean verticalArmMotorInstalled = true;

    // Picker Related Hardware
    public static final boolean pickerPneumaticsInstalled = false;
    public static final boolean stablizerPneumaticsInstalled = false;

    // EveryBot Picker Related Hardware
    public static final boolean everyBotPickerInstalled = true;

    // Arm Sensor Related Hardware
    public static final boolean verticalArmBottomSensorInstalled = false;
    public static final boolean verticalArmMiddleSensorInstalled = true;
    public static final boolean horizontalArmSensorInstalled = true;
}
