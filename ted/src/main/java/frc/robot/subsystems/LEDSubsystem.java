// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
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
import java.util.Map;
import java.util.function.BooleanSupplier;
import java.util.Iterator;

import frc.robot.common.LEDStateAction;
import edu.wpi.first.units.Voltage;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.control.Constants;
import frc.robot.common.LEDState;
import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

public class LEDSubsystem extends SubsystemBase {
    private final CANdle leds;
    
    private final int BUFFER_LENGTH = 144;
    private int startIdx = 0;
        // Creates a new buffer object
      private HashMap <LEDState, LEDStateAction> ledStateActions = new HashMap<LEDState, LEDStateAction>();
      private int blinkPeriodInHertz = 25; // increase this for longer blinks!!!
      private int blinkCounter = 0;
      private boolean currentBlinkState = false;
      private boolean lastBlinkState = false;
       private LEDState currentLEDState = LEDState.Off;


      /**
       * LEDSubsystem
       * @param port PWM port on the roboRIO
        */
      public LEDSubsystem(int canID) {
        this.leds = new CANdle(canID); // initialization of the AdressableLED
        CANdleConfiguration configAll = new CANdleConfiguration();
        configAll.statusLedOffWhenActive = true;
        configAll.disableWhenLOS = false;
        configAll.stripType = LEDStripType.RGBW;
        configAll.brightnessScalar = 0.1;
        configAll.vBatOutputMode = VBatOutputMode.Modulated;
        this.leds.configAllSettings(configAll, 100);

      }

    //Gets the Voltage of VBat as measured by CANdle
    public double getVbat() {
         return leds.getBusVoltage(); 
    }

    //Gets the Voltage of the 5V line as measured by CANdle
    public double get5V() { 
        return leds.get5VRailVoltage(); 
    }

    //Gets the low-side current as measured by CANdle
    public double getCurrent() { 
        return leds.getCurrent(); 
    }

    public double getTemperature() {
         return leds.getTemperature(); 
        }

    public void configBrightness(double percent) { 
        leds.configBrightnessScalar(percent, 0); 
    }

    public void configLos(boolean disableWhenLos) { 
        leds.configLOSBehavior(disableWhenLos, 0); 
    }
    public void configLedType(LEDStripType type) { 
        leds.configLEDType(type, 0); 
    }

    public void configStatusLedBehavior(boolean offWhenActive) { 
        leds.configStatusLedState(offWhenActive, 0); 
    }

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
                  this.setLedStringColor(255,165,0,0);
            }
            else {
                  this.setLedStringColor(0, 0, 0,0);
            }
      }


      private void orangeSolid() {
            this.setLedStringColor(255,165,0,0);
      }


      private void yellowSolid() {
            this.setLedStringColor(150,150,0,0);
      }


      private void greenSolid() {
            this.setLedStringColor(0,200,0,0);
      }


      private void offState() {
            this.setLedStringColor(0,0,0,0);
      }


      private void setLedStringColor(int red, int green, int blue, int white) {
                  this.leds.setLEDs(red, green, blue, white, startIdx, BUFFER_LENGTH);
      }


      private void updateBlinkCounterState() {
            this.lastBlinkState = this.currentBlinkState;
            this.blinkCounter++;
            if(this.blinkCounter % this.blinkPeriodInHertz == 0){
                  this.currentBlinkState = !this.currentBlinkState;
            }
      }
}

