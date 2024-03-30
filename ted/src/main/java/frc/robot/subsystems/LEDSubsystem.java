// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: LEDSubsystem.java
// Intent: Forms a subsystem to control the LEDs
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.control.SubsystemCollection;
import frc.robot.common.LEDState;
import frc.robot.common.LEDStateAction;
//import frc.robot.common.LEDState;


public class LEDSubsystem extends SubsystemBase {
      private final int BUFFER_LENGTH = 150;
      private final AddressableLED leds;
      private final AddressableLEDBuffer buffer;  // Creates a new buffer object
      private SubsystemCollection subsystems;
      private AddressableLEDBuffer ledsOff;
      private ArrayList <LEDStateAction> ledStateActions = new ArrayList<>();
      LEDState nextState;
      private LEDState ledState = LEDState.Off;

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
            leds.stop();
      }

      public void RegisterStateAction(BooleanSupplier shouldTakeAction, LEDState ledState){
            ledStateActions.add(new LEDStateAction(shouldTakeAction, ledState));
      }

      public void periodic(){
            ledState = LEDState.Off;
            for(int i = 0; i < ledStateActions.size(); i++){
                  if(ledState == LEDState.OrangeBlink){
                        noteInIntake();
                  }
                  else if(ledState == LEDState.OrangeSolid){
                        noteAtShooter();
                  }
                  else if(ledState == LEDState.Yellow){
                        shooterRevvedTo80();
                  }
                  else if(ledState == LEDState.Green){
                        shooterRevvedTo90();
                  }
            }

      }

      public void noteInIntake() {
            ledState = LEDState.OrangeBlink;
            for (int i = 0; i < buffer.getLength(); i++) {
                  buffer.setRGB(i, 255,140,0); 
                  buffer.setRGB(i, 0,0,0); 
            }    
      }

      public void noteAtShooter() {
            ledState = LEDState.OrangeSolid;
            for (int i = 0; i < buffer.getLength(); i++) {
                  buffer.setRGB(i, 255,140,0); 
            }   
      }

      public void shooterRevvedTo80() {
            ledState = LEDState.Yellow;
            for (int i = 0; i < buffer.getLength(); i++) {
                  buffer.setRGB(i, 150,150,0); 
            }  
      }

      public void shooterRevvedTo90() {
            ledState = LEDState.Green;
            for (int i = 0; i < buffer.getLength(); i++) {
                  buffer.setRGB(i, 150,150,0); 
            }  
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