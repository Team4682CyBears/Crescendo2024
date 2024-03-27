// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: LEDSubsystem.java
// Intent: Forms a subsystem to control the LEDs
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.control.SubsystemCollection;


public class LEDSubsystem extends SubsystemBase {
      private final int BUFFER_LENGTH = 150;
      private final AddressableLED leds;
      private final AddressableLEDBuffer buffer;  // Creates a new buffer object
      private SubsystemCollection subsystems;

      /**
       * LEDSubsystem
       * @param port PWM port on the roboRIO
       */
      public LEDSubsystem(int port, SubsystemCollection subsystems) {
            // TODO maybe pass in buffer length
            this.subsystems = subsystems;
            leds = new AddressableLED(port); // initialization of the AdressableLED
            leds.setLength(BUFFER_LENGTH); // Sets the LED Strip length once
            buffer = new AddressableLEDBuffer(BUFFER_LENGTH);
            // TODO we start a starting color
            setBuffer(buffer);

            //TODO:SEE IF WE NEED THIS LINE
            leds.start();
      }

      /**
       * Returns the buffer length
       * 
       * @return BUFFER_LENGTH
       */
      public int getBufferLength() {
            return BUFFER_LENGTH;
      }

      public AddressableLEDBuffer getBuffer() {
            return buffer;
      }
      public void setBuffer(AddressableLEDBuffer buffer) {
            leds.setData(buffer);
      }
}