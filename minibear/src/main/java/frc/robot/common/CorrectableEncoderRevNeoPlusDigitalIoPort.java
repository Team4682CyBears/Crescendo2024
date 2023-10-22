// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: public enum CorrectableEncoderRevNeoPlusDigitalIoPort.java
// Intent: Forms an interface that will help keep a motor encoder from drifting with use of external sensor.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.common;

import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DigitalInput;

public class CorrectableEncoderRevNeoPlusDigitalIoPort implements ICorrectableEncoder {

    private RelativeEncoder revNeoMotorEncoder = null;
    private DigitalInput dioStateDevice = null;
    private double encoderTicksAtStateChangeSetPoint = 0.0;
    private boolean lastState = false;
    private boolean motorEncoderPositionReset = false;

    /**
     * Concrete implementation of a pairing between motor encoder and DIO to coordinate resetting of 
     * encoder position based on state change of DIO
     * @param revNeoEncoder - the REV Neo motor encoder
     * @param stateDevice - the DIO state device
     * @param encoderTicksAtStateChange - the set point to use for the motors encoder when the state
     * changes between the current and previous calls into getCurrentEncoderPosition
     * @param sensorTriggeredEncoderInitialPositionTicks - the initial set point to use for the motors
     * encoder when sensor IS triggered to start 
     * @param sensorNotTriggeredEncoderInitialPositionTicks - the initial set point to use for the motors
     * encoder when sensor is NOT triggered to start 
     */
    public CorrectableEncoderRevNeoPlusDigitalIoPort(
        RelativeEncoder revNeoEncoder,
        DigitalInput stateDevice,
        double encoderTicksAtStateChange,
        double sensorTriggeredEncoderInitialPositionTicks,
        double sensorNotTriggeredEncoderInitialPositionTicks) {

        revNeoMotorEncoder = revNeoEncoder;
        dioStateDevice = stateDevice;
        encoderTicksAtStateChangeSetPoint = encoderTicksAtStateChange;

        lastState = dioStateDevice.get();

        // dio state of false is 'triggered' (as in LED is illuminated for 2023 sensors)
        if(lastState == false) {
            this.revNeoMotorEncoder.setPosition(sensorTriggeredEncoderInitialPositionTicks);
        }
        else {
            this.revNeoMotorEncoder.setPosition(sensorNotTriggeredEncoderInitialPositionTicks);
        }
    }

    /**
     * An method intended to be called periodically (as in 1x per 20 ms) that will be used to both
     * correct the motors encoder (when necessary) and also return the current motor encoder position.
     * @return a double representing the motor encoder ticks
     */
    public double getCurrentEncoderPosition() {

        boolean currentState = this.dioStateDevice.get();

        if(currentState != this.lastState) {
            this.revNeoMotorEncoder.setPosition(this.encoderTicksAtStateChangeSetPoint);
            this.lastState = currentState;
            motorEncoderPositionReset = true;
        }

        return this.revNeoMotorEncoder.getPosition();
    }

    /**
     * An  method that will be used to determine if the motor encoder position has ever been reset.
     * @return a boolean to describe if the motor encoder has ever been reset
     */
    public boolean getMotorEncoderEverReset() {
        return motorEncoderPositionReset;
    }

}
