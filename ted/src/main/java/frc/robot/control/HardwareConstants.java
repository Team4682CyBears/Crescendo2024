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

    // ********************************************************************
    // CAN Optimization Constants
    // ctre
    public static final int ctreMotorStatusFramePeriodMilliseconds = 20; // 20ms refresh rate
    public static final int ctreSensorStatusFramePeriodMilliseconds = 30; // 30ms refresh rate
    public static final double ctreMotorStatusFramePeriodFrequencyHertz = 1000.0/(double)HardwareConstants.ctreMotorStatusFramePeriodMilliseconds;
    public static final double ctreSensorStatusFramePeriodFrequencyHertz = 1000.0/(double)HardwareConstants.ctreSensorStatusFramePeriodMilliseconds;
    // playing with fusion
    public static final double playingWithFusionSensorPeriodMilliseconds = 24; // 24ms refresh rate as specified in their documentation (max refresh rate)
}
