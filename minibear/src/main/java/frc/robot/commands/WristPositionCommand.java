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
import frc.robot.common.WristPosition;
import frc.robot.subsystems.WristSubsystem;

public class WristPositionCommand extends CommandBase {

    private WristSubsystem wristSubsystem;
    private WristPosition targetPosition = WristPosition.PositionTwo;

    /**
     * Creates a new SetWristAngleCommand.
     *
     * @param wristSubsystem The subsystem used by this command.
     * @param targetAngle The target angle in degrees for the wrist.
     */
    public WristPositionCommand(WristSubsystem wristSubsystem, WristPosition targetPosition) {
        this.wristSubsystem = wristSubsystem;
        this.targetPosition = targetPosition;
        addRequirements(wristSubsystem);
    }
    
    /**
     * this method is called once per tick until the command is finished. 
     */
    @Override
    public void execute() {
        wristSubsystem.setTargetWristPosition(this.targetPosition);
    }

    /**
     * This method is called once the command has finished. 
     */
    @Override
    public void end(boolean interrupted) {
        if(interrupted || !this.isFinished()){
            wristSubsystem.setWristSpeed(0);
        }
        System.out.println("Wrist Position Target has been reached");
    }

    /**
     * This method returns true if the command is finished
     */
    @Override
    public boolean isFinished() {
        return wristSubsystem.isPositionMovementComplete();
    }

}