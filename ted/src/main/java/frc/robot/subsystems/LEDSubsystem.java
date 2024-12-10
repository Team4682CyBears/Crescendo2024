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
import frc.robot.control.Constants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.LEDState;
import com.ctre.phoenix.led.*;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

public class LEDSubsystem extends SubsystemBase {
  private final CANdle leds;
  private HashMap <LEDState, LEDStateAction> ledStateActions = new HashMap<LEDState, LEDStateAction>();
  private int blinkCounter = 0;
  private int ticksPerSecond = 50;
  private boolean currentBlinkState = false;
  private boolean lastBlinkState = false;
  private LEDState currentLEDState = LEDState.Off;

  /**
  * LEDSubsystem
  * @param canID can id of the CANdle
  */
  public LEDSubsystem(int canID, LEDStripType type) {
    this.leds = new CANdle(canID); // initialization of the AdressableLED
    CANdleConfiguration configAll = new CANdleConfiguration();
    configAll.stripType = type;
    configAll.brightnessScalar = Constants.ledBrightness;
    configAll.vBatOutputMode = VBatOutputMode.Modulated;
    this.leds.configAllSettings(configAll, 100);
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

  //Sets leds to orange blink
  private void orangeBlink() {
    if(this.currentBlinkState) {
      this.setLedStringColor(255,165,0);
    }
    else {
      this.setLedStringColor(0, 0, 0);
    }
  }

  //Sets leds to orange solid
  private void orangeSolid() {
    this.setLedStringColor(255,165,0);
  }

  //Sets leds to yellow solid
  private void yellowSolid() {
    this.setLedStringColor(150,150,0);
  }

  //Sets leds to green solid
  private void greenSolid() {
    this.setLedStringColor(0,200,0);
  }

  //Turns off leds
  private void offState() {
    this.setLedStringColor(0,0,0);
  }

  //Sets leds color
  private void setLedStringColor(int red, int green, int blue) {
    this.leds.setLEDs(red, green, blue, 0, Constants.ledStartIdx, Constants.ledLength);
  }

  //Updates the blink state of leds
  private void updateBlinkCounterState() {
    this.lastBlinkState = this.currentBlinkState;
    this.blinkCounter++;
    if(this.blinkCounter % (ticksPerSecond / Constants.ledBlinkFrquencyInHertz) == 0){
      this.currentBlinkState = !this.currentBlinkState;
    }
  }
}

