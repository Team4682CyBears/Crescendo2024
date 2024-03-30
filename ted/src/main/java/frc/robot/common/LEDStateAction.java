package frc.robot.common;

import java.util.function.BooleanSupplier;


public class LEDStateAction {
    BooleanSupplier shouldTakeAction;
    LEDState ledState;
    public LEDStateAction(BooleanSupplier shouldTakeAction, LEDState ledState){
        this.shouldTakeAction = shouldTakeAction;
        this.ledState = ledState;
    }

    public BooleanSupplier getShouldTakeAction(){
        return this.shouldTakeAction;
    }
    public LEDState getLedState(){
        return this.ledState;
    }
}
