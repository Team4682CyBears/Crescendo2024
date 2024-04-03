package frc.robot.common;

import java.util.function.BooleanSupplier;

/**
 * Class to encapsulate the LED state action
 */
public class LEDStateAction {
    private BooleanSupplier shouldTakeAction;
    private LEDState ledState;

    /**
     * A wrapper class to encapsulate both the state and its 
     * @param shouldTakeAction - When true the state should be enabled, when false the state is not triggered
     * @param ledState - the state associated with the double supplier
     */
    public LEDStateAction(BooleanSupplier shouldTakeAction, LEDState ledState){
        this.shouldTakeAction = shouldTakeAction;
        this.ledState = ledState;
    }

    /**
     * Get at the underlying double supplier
     * @return
     */
    public BooleanSupplier getShouldTakeAction(){
        return this.shouldTakeAction;
    }

    /**
     * Getter method for the state associated with the double supplier
     * @return - The LED state
     */
    public LEDState getLedState(){
        return this.ledState;
    }
}
