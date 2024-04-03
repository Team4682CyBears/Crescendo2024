// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: LEDSubsystem.java
// Intent: Forms a subsystem to control the LEDs
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.function.BooleanSupplier;

import edu.wpi.first.math.Pair;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.LEDState;
import frc.robot.common.LEDStateAction;

public class LEDSubsystem extends SubsystemBase {

      private final int BUFFER_LENGTH = 144;
      private final AddressableLED leds;
      private final AddressableLEDBuffer buffer;  // Creates a new buffer object
      private HashMap <LEDState, LEDStateAction> ledStateActions = new HashMap<LEDState, LEDStateAction>();
      private int blinkPeriodInHertz = 10; // increase this for longer blinks!!!
      private int blinkCounter = 0;
      private boolean lastBlinkState = false;
      private boolean currentBlinkState = false;
      private LEDState currentLEDState = LEDState.Off;

      /**
       * LEDSubsystem
       * @param port PWM port on the roboRIO
       */
      public LEDSubsystem(int port) {

            leds = new AddressableLED(port); // initialization of the AdressableLED
            buffer = new AddressableLEDBuffer(BUFFER_LENGTH);
            leds.setLength(buffer.getLength()); // Sets the LED Strip length once

            leds.setData(buffer);
            leds.start();
      }

      /**
       * Register a state action
       * @param ledState
       * @param shouldTakeAction
       */
      public void registerStateAction(LEDState ledState, BooleanSupplier shouldTakeAction){
            if(ledStateActions.containsKey(ledState)) {
                  ledStateActions.remove(ledState);
            }
            ledStateActions.put(ledState, new LEDStateAction(ledState, shouldTakeAction));
      }

      public void periodic() {

            // figure out if the blink should be on or off now
            this.updateBlinkCounterState();

            // iterate through all of the states to get most recent action for each that should be taken
            HashMap <LEDState, Boolean> currentActions = new HashMap<LEDState, Boolean>();
            Iterator<Map.Entry<LEDState, LEDStateAction>> iter = this.ledStateActions.entrySet().iterator();
            while (iter.hasNext()) {
                  Map.Entry<LEDState, LEDStateAction> entry = iter.next();
                  currentActions.put(entry.getKey(), entry.getValue().getRecentState());
            }

            // find the states in precidence order
            LEDState targetLedState = LEDState.Off;

            if(currentActions.containsKey(LEDState.Green) && currentActions.get(LEDState.Green).booleanValue()) {
                  targetLedState = LEDState.Green;
            }
            else if(currentActions.containsKey(LEDState.Yellow) && currentActions.get(LEDState.Yellow).booleanValue()) {
                  targetLedState = LEDState.Yellow;
            }
            else if(currentActions.containsKey(LEDState.OrangeSolid) && currentActions.get(LEDState.OrangeSolid).booleanValue()) {
                  targetLedState = LEDState.OrangeSolid;
            }
            else if(currentActions.containsKey(LEDState.OrangeBlink) && currentActions.get(LEDState.OrangeBlink).booleanValue()) {
                  targetLedState = LEDState.OrangeBlink;
            }

            // update the LED state when the target state has changed
            if(this.currentLEDState != targetLedState) {
                  this.currentLEDState = targetLedState;

                  if(this.currentLEDState == LEDState.Green) {
                        this.greenSolid();
                  }
                  else if(this.currentLEDState == LEDState.Yellow) {
                        this.yellowSolid();
                  }
                  else if(this.currentLEDState == LEDState.OrangeSolid) {
                        this.orangeSolid();
                  }
                  else if(this.currentLEDState == LEDState.OrangeBlink) {
                        this.orangeBlink();
                  }
                  else if(this.currentLEDState == LEDState.Off) {
                        this.offState();
                  }
                  System.out.println("**** UPDATING LED STATE TO " + this.currentLEDState.toString());
            }
            else if(this.lastBlinkState != this.currentBlinkState && this.currentLEDState == LEDState.OrangeBlink) {
                  System.out.println("**** BLINKING LED STATE TO " + this.currentLEDState.toString());
                  this.orangeBlink();
            }
      }

      private void orangeBlink() {
            if(this.currentBlinkState) {
                  this.setLedStringColor(255,165,0); 
            }
            else {
                  this.setLedStringColor(0, 0, 0);
            }
      }

      private void orangeSolid() {
            this.setLedStringColor(255,165,0); 
      }

      private void yellowSolid() {
            this.setLedStringColor(150,150,0); 
      }

      private void greenSolid() {
            this.setLedStringColor(150,150,0); 
      }

      private void offState() {
            this.setLedStringColor(0,0,0);
      }

      private void setLedStringColor(int red, int green, int blue) {
            for (int i = 0; i < buffer.getLength(); i++) {
                  buffer.setRGB(i, red, green, blue); 
            }  
            this.leds.setData(this.buffer);
      }

      private void updateBlinkCounterState() {
            this.lastBlinkState = this.currentBlinkState;
            this.blinkCounter++;
            if(this.blinkCounter % this.blinkPeriodInHertz == 0){
                  this.currentBlinkState = !this.currentBlinkState;
            }
      }
}