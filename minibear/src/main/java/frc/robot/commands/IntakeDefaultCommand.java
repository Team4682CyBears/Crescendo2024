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
import frc.robot.common.WristPosition;
import frc.robot.control.ManualInputInterfaces;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;

import java.util.function.DoubleSupplier;

public class IntakeDefaultCommand extends CommandBase {

    private IntakeSubsystem intakeSub;
    private WristSubsystem wristSub;
    private DoubleSupplier uptakeSupplier;
    private DoubleSupplier expelSupplier;
    private final double inputThreshold = 0.1;

    /**
     * Constructor for DefaultEveryBotPickerCommand
     * @param everyBotPickerSubsystem - the subsystem for the everybot picker
     * @param uptakeSupplier - trigger uptake value
     * @param expelSupplier - trigger expel value
     */
    public IntakeDefaultCommand(WristSubsystem wristSubsystem,
                                IntakeSubsystem intakeSubsystem,
                                DoubleSupplier uptakeInputSupplier,
                                DoubleSupplier expelInputSupplier) {
        this.intakeSub = intakeSubsystem;
        this.wristSub = wristSubsystem;
        this.uptakeSupplier = uptakeInputSupplier;
        this.expelSupplier = expelInputSupplier;

        addRequirements(this.intakeSub);
    }

    @Override
    public void execute() {
        double uptakeValue = this.uptakeSupplier.getAsDouble();
        double expelValue = this.expelSupplier.getAsDouble();

        WristPosition targetWristPosition = this.wristSub.getTargetWristPosition();
        if(targetWristPosition == WristPosition.PickUp) {
            uptakeValue *= Constants.INTAKE_SPEED; 
            expelValue *= Constants.SHOOT_SPEED_0;
        }
        else if(targetWristPosition == WristPosition.PositionOne) {
            uptakeValue *= Constants.INTAKE_SPEED; 
            expelValue *= Constants.SHOOT_SPEED_1;
        }
        else if(targetWristPosition == WristPosition.PositionTwo) {
            uptakeValue *= Constants.INTAKE_SPEED; 
            expelValue *= Constants.SHOOT_SPEED_2;
        }
            // ... add more as needed

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

        this.intakeSub.setIntakeRelativeSpeed(inputValue);
    }

    @Override
    public void end(boolean interrupted) {
        this.intakeSub.setIntakeRelativeSpeed(0.0);
    }
}