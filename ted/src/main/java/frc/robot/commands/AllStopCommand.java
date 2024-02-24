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
        if(this.subsystems.isDriveTrainSubsystemAvailable()) {
            addRequirements(this.subsystems.getDriveTrainSubsystem());
        }
        if(this.subsystems.isFeederSubsystemAvailable()) {
            addRequirements(this.subsystems.getFeederSubsystem());
        }
        if(this.subsystems.isIntakeSubsystemAvailable()) {
            addRequirements(this.subsystems.getIntakeSubsystem());
        }
        if(this.subsystems.isShooterSubsystemAvailable()) {
            addRequirements(this.subsystems.getShooterSubsystem());
        }
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if(this.subsystems.isDriveTrainSubsystemAvailable()) {
            this.subsystems.getDriveTrainSubsystem().drive(new ChassisSpeeds(0.0,0.0,0.0));
            this.subsystems.getDriveTrainSubsystem().setSwerveDriveMode(SwerveDriveMode.NORMAL_DRIVING);
        }
        if(this.subsystems.isFeederSubsystemAvailable()) {
            this.subsystems.getFeederSubsystem().setAllStop();
        }
        if(this.subsystems.isIntakeSubsystemAvailable()) {
            this.subsystems.getIntakeSubsystem().setAllStop();
        }
        if(this.subsystems.isShooterSubsystemAvailable()) {
            this.subsystems.getShooterSubsystem().setAllStop();
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