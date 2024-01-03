// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: public enum CorrectableEncoder.java
// Intent: Forms an interface that will help keep a motor encoder from drifting with use of external sensor.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.common;

public interface ICorrectableEncoder {
    
    /**
     * An interface method that will be used to both correct (when necessary) the motors encoder
     * and also return the current motor encoder position.
     * @return a double representing the motor encoder ticks
     */
    public double getCurrentEncoderPosition();

    /**
     * An interface method that will be used to determine if the motor encoder position has ever been reset.
     * @return a boolean to describe if the motor encoder has ever been reset
     */
    public boolean getMotorEncoderEverReset();
}
