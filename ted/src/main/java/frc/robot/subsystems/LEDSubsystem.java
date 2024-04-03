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
import java.util.HashMap;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.control.SubsystemCollection;
import frc.robot.common.LEDState;
import frc.robot.common.LEDStateAction;
//import frc.robot.common.LEDState;


public class LEDSubsystem extends SubsystemBase {
      private final int BUFFER_LENGTH = 144;
      private final AddressableLED leds;
      private final AddressableLEDBuffer buffer;  // Creates a new buffer object
      private HashMap <LEDState, LEDStateAction> ledStateActions = new HashMap<LEDState, LEDStateAction>();
      private int blinkPeriodInHertz = 10; // increase this for longer blinks!!!
      private int blinkCounter = 0;
      private boolean currentBlinkState = false;


      /**
       * LEDSubsystem
       * @param port PWM port on the roboRIO
       */
      public LEDSubsystem(int port) {

            leds = new AddressableLED(port); // initialization of the AdressableLED
            leds.setLength(BUFFER_LENGTH); // Sets the LED Strip length once
            buffer = new AddressableLEDBuffer(BUFFER_LENGTH);

            //TODO:SEE IF WE NEED THIS LINE
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

      public void periodic(){

            this.updateBlinkCounterState();

            // LEDState.Off
            this.offState();

            // LEDState.OrangeBlink
            if(ledStateActions.containsKey(LEDState.OrangeBlink)){
                  LEDStateAction action = ledStateActions.get(LEDState.OrangeBlink);
                  if(action.getShouldTakeAction().getAsBoolean()) {
                        System.out.println("Setting led state to " + LEDState.OrangeBlink.toString());
                        this.orangeBlink();
                  }
            }

            // LEDState.OrangeSolid
            if(ledStateActions.containsKey(LEDState.OrangeSolid)){
                  LEDStateAction action = ledStateActions.get(LEDState.OrangeSolid);
                  if(action.getShouldTakeAction().getAsBoolean()) {
                        System.out.println("Setting led state to " + LEDState.OrangeSolid.toString());
                        this.orangeSolid();
                  }
            }

            // LEDState.Yellow
            if(ledStateActions.containsKey(LEDState.Yellow)){
                  LEDStateAction action = ledStateActions.get(LEDState.Yellow);
                  if(action.getShouldTakeAction().getAsBoolean()) {
                        System.out.println("Setting led state to " + LEDState.Yellow.toString());
                        this.yellowSolid();
                  }
            }

            // LEDState.Green
            if(ledStateActions.containsKey(LEDState.Green)){
                  LEDStateAction action = ledStateActions.get(LEDState.Green);
                  if(action.getShouldTakeAction().getAsBoolean()) {
                        System.out.println("Setting led state to " + LEDState.Green.toString());
                        this.greenSolid();
                  }
            }

      }

      private void orangeBlink() {
            boolean localBlinkState = this.currentBlinkState;
            for (int i = 0; i < buffer.getLength(); i++) {
                  if(localBlinkState) {
                        buffer.setRGB(i, 255,140,0); 
                  }
                  else {
                        buffer.setRGB(i, 0, 0, 0);
                  }
            } 
      }

      private void orangeSolid() {
            for (int i = 0; i < buffer.getLength(); i++) {
                  buffer.setRGB(i, 255,140,0); 
            }   
      }

      private void yellowSolid() {
            for (int i = 0; i < buffer.getLength(); i++) {
                  buffer.setRGB(i, 150,150,0); 
            }  
      }

      private void greenSolid() {
            for (int i = 0; i < buffer.getLength(); i++) {
                  buffer.setRGB(i, 150,150,0); 
            }  
      }

      private void offState() {
            for (int i = 0; i < buffer.getLength(); i++) {
                  buffer.setRGB(i, 0,0,0); 
            }  
      }

      private void updateBlinkCounterState() {
            this.blinkCounter++;
            if(this.blinkCounter % this.blinkPeriodInHertz == 0){
                  this.currentBlinkState = !this.currentBlinkState;
            }
      }
}