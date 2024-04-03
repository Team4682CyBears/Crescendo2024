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
import java.util.function.BooleanSupplier;
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
      private boolean currentBlinkState = false;
      private boolean lastBlinkState = false;
      private LEDState lastLEDState = LEDState.Off;

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
            ledStateActions.put(ledState, new LEDStateAction(shouldTakeAction, ledState));
      }

      public void periodic() {

            boolean hasPreviousColorState = false;
            LEDState currLedState = LEDState.Off;

            this.updateBlinkCounterState();

            // LEDState.Green
            if(!hasPreviousColorState && ledStateActions.containsKey(LEDState.Green)){
                  LEDStateAction action = ledStateActions.get(LEDState.Green);
                  if(action.getShouldTakeAction().getAsBoolean()) {
                        currLedState = LEDState.Green;
                        hasPreviousColorState = true;
                        if(this.doUpdateOfRequestedLedColorState(currLedState)) {
                              System.out.println("**** UPDATING LED STATE TO " + currLedState.toString());
                              this.greenSolid();
                        }
                  }
            }

            // LEDState.Yellow
            if(!hasPreviousColorState && ledStateActions.containsKey(LEDState.Yellow)){
                  LEDStateAction action = ledStateActions.get(LEDState.Yellow);
                  if(action.getShouldTakeAction().getAsBoolean()) {
                        currLedState = LEDState.Yellow;
                        hasPreviousColorState = true;
                        if(this.doUpdateOfRequestedLedColorState(currLedState)) {
                              System.out.println("**** UPDATING LED STATE TO " + currLedState.toString());
                              this.yellowSolid();
                        }
                  }
            }

            // LEDState.OrangeSolid
            if(!hasPreviousColorState && ledStateActions.containsKey(LEDState.OrangeSolid)){
                  LEDStateAction action = ledStateActions.get(LEDState.OrangeSolid);
                  if(action.getShouldTakeAction().getAsBoolean()) {
                        currLedState = LEDState.OrangeSolid;
                        hasPreviousColorState = true;
                        if(this.doUpdateOfRequestedLedColorState(currLedState)) {
                              System.out.println("**** UPDATING LED STATE TO " + currLedState.toString());
                              this.orangeSolid();
                        }
                  }
            }

            // LEDState.OrangeBlink
            if(!hasPreviousColorState && ledStateActions.containsKey(LEDState.OrangeBlink)){
                  LEDStateAction action = ledStateActions.get(LEDState.OrangeBlink);
                  if(action.getShouldTakeAction().getAsBoolean()) {
                        currLedState = LEDState.OrangeBlink;
                        hasPreviousColorState = true;
                        boolean theCurrentBlinkState = this.currentBlinkState;
                        if(this.doUpdateOfRequestedLedColorState(currLedState) || theCurrentBlinkState != this.lastBlinkState) {
                              this.lastBlinkState = theCurrentBlinkState;
                              System.out.println("**** UPDATING LED STATE TO " + currLedState.toString());
                              this.orangeBlink(theCurrentBlinkState);
                        }
                  }
            }

            // LEDState.Off
            if(!hasPreviousColorState && currLedState == LEDState.Off) {
                  currLedState = LEDState.Off;
                  hasPreviousColorState = true;
                  if(this.doUpdateOfRequestedLedColorState(currLedState)) {
                        System.out.println("No other LED state requested!!");
                        System.out.println("**** UPDATING LED STATE TO " + currLedState.toString());
                        this.offState();
                  }
            }

      }

      private void orangeBlink(boolean localBlinkState) {
            for (int i = 0; i < buffer.getLength(); i++) {
                  if(localBlinkState) {
                        buffer.setRGB(i, 255,165,0); 
                  }
                  else {
                        buffer.setRGB(i, 0, 0, 0);
                  }
            }
            this.leds.setData(this.buffer);
      }

      private void orangeSolid() {
            for (int i = 0; i < buffer.getLength(); i++) {
                  buffer.setRGB(i, 255,165,0); 
            }   
            this.leds.setData(this.buffer);
      }

      private void yellowSolid() {
            for (int i = 0; i < buffer.getLength(); i++) {
                  buffer.setRGB(i, 150,150,0); 
            }  
            this.leds.setData(this.buffer);
      }

      private void greenSolid() {
            for (int i = 0; i < buffer.getLength(); i++) {
                  buffer.setRGB(i, 150,150,0); 
            }  
            this.leds.setData(this.buffer);
      }

      private void offState() {
            for (int i = 0; i < buffer.getLength(); i++) {
                  buffer.setRGB(i, 0,0,0); 
            }  
            this.leds.setData(this.buffer);
      }

      private void updateBlinkCounterState() {
            this.blinkCounter++;
            if(this.blinkCounter % this.blinkPeriodInHertz == 0){
                  this.currentBlinkState = !this.currentBlinkState;
            }
      }

      private boolean doUpdateOfRequestedLedColorState(LEDState requestedState) {
            boolean changeState = false;
            if(requestedState != this.lastLEDState) {
                  changeState = true;
                  this.lastLEDState = requestedState;
            }
            return changeState;
      }
}