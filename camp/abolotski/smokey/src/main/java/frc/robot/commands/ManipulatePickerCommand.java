// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: ManipulatePickerCommand.java
// Intent: Forms a command to open or close the picker.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PickerSubsystem;

public class ManipulatePickerCommand extends CommandBase {

    private PickerSubsystem thePicker = null;
    private boolean isPickerOpen = true;

    /**
     * Constructor to open or close the picker
     * @param picker - the picker subsystems
     * @param isOpen - if true the picker will be moved to the open position, else it is moved to the closed position
     */
    public ManipulatePickerCommand(PickerSubsystem picker, boolean isOpen) {
        thePicker = picker;
        isPickerOpen = isOpen;
        addRequirements(thePicker);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if(isPickerOpen) {
            thePicker.retractHorizontalPosition();
            thePicker.retractVerticalPosition();
        }
        else {
            thePicker.deployHorizontalPosition();
            thePicker.deployHorizontalPosition();
        }
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished(){
        return true;
    }
}