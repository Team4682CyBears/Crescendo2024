// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: WristDefaultCommand.java
// Intent: Forms a command to move the wrist to preset positions.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.WristPosition;
import frc.robot.subsystems.WristSubsystem;

public class WristDefaultCommand extends CommandBase {


    private final double inputThreshold = 0.1;
    private WristSubsystem wristSubsystem;
    private double targetAngle;

    /**
     * Creates a new SetWristAngleCommand.
     *
     * @param wristSubsystem The subsystem used by this command.
     * @param targetAngle The target angle in degrees for the wrist.
     */

    public WristDefaultCommand(WristSubsystem wristSubsystem, double targetAngle) {
        this.wristSubsystem = wristSubsystem;
        this.targetAngle = targetAngle;
        addRequirements(wristSubsystem);
    }


    
    @Override
    public void execute() {
        
        wristSubsystem.setPickerAngle(targetAngle);
        
        // Update the state based on target angle (this assumes unique angles for each position)
        if(targetAngle == Constants.WRIST_ANGLE_PICKUP) {
            Constants.currentWristPosition = WristPosition.PICKUP;
        } else if(targetAngle == Constants.WRIST_ANGLE_1) {
            Constants.currentWristPosition = WristPosition.POSITION_1;
        } else if(targetAngle == Constants.WRIST_ANGLE_2) {
            Constants.currentWristPosition = WristPosition.POSITION_2;
        }
        
    }

    @Override
    public void end(boolean interrupted) {
        wristSubsystem.setPickerRelativeSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return true;
    }


}