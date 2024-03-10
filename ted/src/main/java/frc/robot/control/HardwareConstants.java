// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: HardwareConstants.java
// Intent: Forms key HARDWARE-only constants required for this robot.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.control;

public class HardwareConstants {

    // *******************************************************************
    // CTRE motor constants
    public static final double ctreStatorCurrentMaximumAmps = 100.0;
    public static final double ctreSupplyCurrentMaximumAmps = 50.0;
    public static final double ctreSupplyVoltageTimeConstant = 0.02;
   
    // *******************************************************************
    // shooter outfeed motor constants
    public static final double shooterOutfeedStatorCurrentMaximumAmps = HardwareConstants.ctreStatorCurrentMaximumAmps;
    public static final double shooterOutfeedSupplyCurrentMaximumAmps = HardwareConstants.ctreSupplyCurrentMaximumAmps;
    public static final double shooterOutfeedSupplyVoltageTimeConstant = HardwareConstants.ctreSupplyVoltageTimeConstant;
      
    // *******************************************************************
    // shooter angle motor constants
    public static final double shooterAngleStatorCurrentMaximumAmps = HardwareConstants.ctreStatorCurrentMaximumAmps;
    public static final double shooterAngleSupplyCurrentMaximumAmps = HardwareConstants.ctreSupplyCurrentMaximumAmps;
    public static final double shooterAngleSupplyVoltageTimeConstant = HardwareConstants.ctreSupplyVoltageTimeConstant;

}
