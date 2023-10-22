// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: DefaultEveryBotPickerCommand.java
// Intent: Forms a command to uptake/expel the cargo from the every bot picker.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.EveryBotPickerSubsystem;

import java.util.function.DoubleSupplier;

public class EveryBotPickerDefaultCommand extends CommandBase {

    private EveryBotPickerSubsystem everyBotPickerSub;
    private DoubleSupplier uptakeSupplier;
    private DoubleSupplier expelSupplier;
    private final double inputThreshold = 0.1;

    /**
     * Constructor for DefaultEveryBotPickerCommand
     * @param everyBotPickerSubsystem - the subsystem for the everybot picker
     * @param uptakeSupplier - trigger uptake value
     * @param expelSupplier - trigger expel value
     */
    public EveryBotPickerDefaultCommand(EveryBotPickerSubsystem everyBotPickerSubsystem,
                               DoubleSupplier uptakeInputSupplier,
                               DoubleSupplier expelInputSupplier) {
        this.everyBotPickerSub = everyBotPickerSubsystem;
        this.uptakeSupplier = uptakeInputSupplier;
        this.expelSupplier = expelInputSupplier;

        addRequirements(this.everyBotPickerSub);
    }

    @Override
    public void execute() {
        double uptakeValue = this.uptakeSupplier.getAsDouble();
        double expelValue = this.expelSupplier.getAsDouble();

        switch(Constants.currentWristPosition) {
            case PICKUP:
                uptakeValue *= Constants.INTAKE_SPEED; 
                expelValue *= Constants.SHOOT_SPEED_0;
                break;
            case POSITION_1:
                uptakeValue *= Constants.INTAKE_SPEED; 
                expelValue *= Constants.SHOOT_SPEED_1;
                break;
            case POSITION_2:
                uptakeValue *= Constants.INTAKE_SPEED; 
                expelValue *= Constants.SHOOT_SPEED_2;
                break;
            // ... add more as needed
        }

        //clean inputs
        double uptakeAbsValue = Math.abs(uptakeValue);
        double expelAbsValue = Math.abs(expelValue);
        double inputValue = 0.0;

        // we will always favor uptake over expel as this will help avoid accidental drops
        if(uptakeAbsValue >= this.inputThreshold) {
            inputValue = uptakeValue;
        }
        else if (expelAbsValue > this.inputThreshold && expelAbsValue > uptakeAbsValue) {
            inputValue = expelValue;
        }

        this.everyBotPickerSub.setPickerRelativeSpeed(inputValue);
    }

    @Override
    public void end(boolean interrupted) {
        this.everyBotPickerSub.setPickerRelativeSpeed(0.0);
    }
}