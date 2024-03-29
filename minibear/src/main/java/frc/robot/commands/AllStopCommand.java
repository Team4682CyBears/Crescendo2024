// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: AllStopCommand.java
// Intent: Forms a command to stop all subsystems.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.control.SubsystemCollection;

/**
 * Class to form a command to stop all subsystems
 */
public class AllStopCommand extends CommandBase {
    private final SubsystemCollection subsystems;

    /**
     * Constructor to cause all subsystems to halt movements
     * 
     * @param collection - the collection of subsystems
     */
    public AllStopCommand(SubsystemCollection collection) {
        subsystems = collection;
    }

    @Override
    public void initialize() {
    }

    
    @Override
    public void execute() {
        if (this.subsystems.getIntakeSubsystem() != null) {
            this.subsystems.getIntakeSubsystem().setIntakeRelativeSpeed(0);
        }
        if (this.subsystems.getWristSubsystem() != null) {
            this.subsystems.getWristSubsystem().setWristSpeed(0);
        }   
    }

    @Override
    public void end(boolean interrupted) {
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}