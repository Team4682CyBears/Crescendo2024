// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: AllStopCommand.java
// Intent: Forms a command to stop all subsystems.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.control.SubsystemCollection;
import frc.robot.control.SwerveDriveMode;

/**
 * Class to form a command to stop all subsystems
 */
public class AllStopCommand extends Command {
    private final SubsystemCollection subsystems;

    /**
     * Constructor to cause all subsystems to halt movements
     * @param collection - the collection of subsystems
     */
    public AllStopCommand(SubsystemCollection collection) {
        subsystems = collection;
        if(this.subsystems.getDriveTrainSubsystem() != null) {
            addRequirements(this.subsystems.getDriveTrainSubsystem());
        }
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if(this.subsystems.getDriveTrainSubsystem() != null) {
            this.subsystems.getDriveTrainSubsystem().drive(new ChassisSpeeds(0.0,0.0,0.0));
            this.subsystems.getDriveTrainSubsystem().setSwerveDriveMode(SwerveDriveMode.NORMAL_DRIVING);
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