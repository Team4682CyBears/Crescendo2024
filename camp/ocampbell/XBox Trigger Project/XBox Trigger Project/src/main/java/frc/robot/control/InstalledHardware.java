package frc.robot.control;
// ***********************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// File: InstalledHardware.java
// Intent: Forms a listing of switches that will help to debug code better as hardware is available (or not available).
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 


/**
 * A class devoted to installed hardware constants.  Use this to decide if hardware is enabled on the robot or not.
 * All developers to use this to protect their subsystems and commands from using hardware that is not actually installed
 * on the robot at hand.  Used to assist in development stages and make it easier to quickly remove a piece of hardware
 * from the robot.
 */

public class InstalledHardware
{

    // Basic Hardware
    public static final boolean powerDistributionPanelInstalled = true;
    
    // Onboard Hardware - Orentation/Navigation Hardware
    public static final boolean wifiRadioInstalled = true;

    // External Input Hardware
    public static final boolean driverXboxControllerInstalled = true;
    public static final boolean coDriverXboxControllerInstalled = false;

    // Controlled Falcon
    public static final boolean controlledFalconInstalled = true;



}
