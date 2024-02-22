// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: IntakeDefaultCommand.java
// Intent: Forms a command to uptake/expel the cargo.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.common.WristPosition;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;

import java.util.function.DoubleSupplier;

public class IntakeDefaultCommand extends CommandBase {

    private IntakeSubsystem intakeSub;
    private WristSubsystem wristSub;
    private DoubleSupplier uptakeSupplier;
    private DoubleSupplier expelSupplier;
    private final double inputThreshold = 0.1;
    // when true, rotates the cube in the intake. 
    // when false, normal operation 
    private boolean ponderMode = false; 

    // TODO restructure this to take in both a speed 
    // and an enum that controls the direction of intake or expel
    /**
     * Constructor for IntakeDefaultCommand wit ponder option
     * @param wristSubsystem - the subsystem for the wrist
     * @param intakeSubsystem - the subsystem for the intake
     * @param uptakeSupplier - trigger uptake value
     * @param expelSupplier - trigger expel value
     */
    public IntakeDefaultCommand(WristSubsystem wristSubsystem,
                                IntakeSubsystem intakeSubsystem,
                                DoubleSupplier uptakeInputSupplier,
                                DoubleSupplier expelInputSupplier,
                                boolean ponderMode) {
        this.intakeSub = intakeSubsystem;
        this.wristSub = wristSubsystem;
        this.uptakeSupplier = uptakeInputSupplier;
        this.expelSupplier = expelInputSupplier;
        this.ponderMode = ponderMode;

        addRequirements(this.intakeSub);
        addRequirements(this.wristSub);
    }

    /**
     * Constructor for IntakeDefaultCommand without ponder option
     * @param wristSubsystem - the subsystem for the wrist
     * @param intakeSubsystem - the subsystem for the intake
     * @param uptakeSupplier - trigger uptake value
     * @param expelSupplier - trigger expel value
     */
    public IntakeDefaultCommand(WristSubsystem wristSubsystem,
                                IntakeSubsystem intakeSubsystem,
                                DoubleSupplier uptakeInputSupplier,
                                DoubleSupplier expelInputSupplier) {
        this(wristSubsystem, intakeSubsystem, uptakeInputSupplier, expelInputSupplier, false);
    }

    /**
     * this function runs once per tick until the command is finished
     */
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
        else if(targetWristPosition == WristPosition.PositionThree) {
            uptakeValue *= Constants.INTAKE_SPEED; 
            expelValue *= Constants.SHOOT_SPEED_3;
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
            inputValue = expelValue * -1;
        }

        if (this.ponderMode) {
            this.intakeSub.setIntakePonder(inputValue);
        }
        else {
            this.intakeSub.setIntakeRelativeSpeed(inputValue);
        }
 
    }

    /**
     * this method is called when the command is finished or interruped
     */
    @Override
    public void end(boolean interrupted) {
        this.intakeSub.stopMotors();
    }
}