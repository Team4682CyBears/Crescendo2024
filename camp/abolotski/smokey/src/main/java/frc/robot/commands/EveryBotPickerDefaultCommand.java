// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: DefaultEveryBotPickerCommand.java
// Intent: Forms a command to uptake/expell the cargo from the every bot picker.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EveryBotPickerSubsystem;

import java.util.function.DoubleSupplier;

public class EveryBotPickerDefaultCommand extends CommandBase {

    private EveryBotPickerSubsystem everyBotPickerSub;
    private DoubleSupplier uptakeSupplier;
    private DoubleSupplier expellSupplier;
    private final double inputThreshold = 0.1;

    /**
     * Ccnstructor for DefaultEveryBotPickerCommand
     * @param everyBotPickerSubsystem - the subsystem for the everybot picker
     * @param uptakeSupplier - trigger uptake value
     * @param expellSupplier - trigger expell value
     */
    public EveryBotPickerDefaultCommand(EveryBotPickerSubsystem everyBotPickerSubsystem,
                               DoubleSupplier uptakeInputSupplier,
                               DoubleSupplier expellInputSupplier) {
        this.everyBotPickerSub = everyBotPickerSubsystem;
        this.uptakeSupplier = uptakeInputSupplier;
        this.expellSupplier = expellInputSupplier;

        addRequirements(this.everyBotPickerSub);
    }

    @Override
    public void execute() {
        double uptakeValue = this.uptakeSupplier.getAsDouble();
        double expellValue = this.expellSupplier.getAsDouble();
        double uptakeAbsValue = Math.abs(uptakeValue);
        double expellAbsValue = Math.abs(expellValue);
        double inputValue = 0.0;
        // we will always favor uptake over expell as this will help avoid accidental drops
        if(uptakeAbsValue >= this.inputThreshold) {
            inputValue = uptakeValue;
        }
        else if (expellAbsValue > this.inputThreshold && expellAbsValue > uptakeAbsValue) {
            inputValue = expellValue;
        }
        this.everyBotPickerSub.setPickerRelativeSpeed(inputValue);
    }

    @Override
    public void end(boolean interrupted) {
        this.everyBotPickerSub.setPickerRelativeSpeed(0.0);
    }
}