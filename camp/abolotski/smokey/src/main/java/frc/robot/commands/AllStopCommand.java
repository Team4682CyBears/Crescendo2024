// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: AllStopCommand.java
// Intent: Forms a command to stop all subsystems.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.control.SubsystemCollection;
import frc.robot.control.SwerveDriveMode;

/**
 * Class to form a command to stop all subsystems
 */
public class AllStopCommand extends CommandBase {
    private final SubsystemCollection subsystems;

    /**
     * Constructor to cause all subsystems to halt movements
     * @param collection - the collection of subsystems
     */
    public AllStopCommand(SubsystemCollection collection) {
        subsystems = collection;
        if(this.subsystems.getArmSubsystem() != null) {
            addRequirements(this.subsystems.getArmSubsystem());
        }
        if(this.subsystems.getDriveTrainSubsystem() != null) {
            addRequirements(this.subsystems.getDriveTrainSubsystem());
        }
        if(this.subsystems.getPickerSubsystem() != null) {
            addRequirements(this.subsystems.getPickerSubsystem());
        }
        if(this.subsystems.getPickerSubsystem() != null) {
            addRequirements(this.subsystems.getPickerSubsystem());
        }
        if(this.subsystems.getStabilizerSubsystem() != null) {
            addRequirements(this.subsystems.getStabilizerSubsystem());
        }
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        if(this.subsystems.getArmSubsystem() != null) {
            this.subsystems.getArmSubsystem().setArmSpeeds(0.0, 0.0);
        }
        if(this.subsystems.getDriveTrainSubsystem() != null) {
            this.subsystems.getDriveTrainSubsystem().drive(new ChassisSpeeds(0.0,0.0,0.0));
            this.subsystems.getDriveTrainSubsystem().setSwerveDriveMode(SwerveDriveMode.NORMAL_DRIVING);
        }
        if(this.subsystems.getPickerSubsystem() != null) {
            // safer to be deployed as we are less likely to drop a game piece this way
            this.subsystems.getPickerSubsystem().deployHorizontalPosition();
            this.subsystems.getPickerSubsystem().deployVerticalPosition();
        }
        if(this.subsystems.getStabilizerSubsystem() != null) {
            this.subsystems.getStabilizerSubsystem().retractPosition();
        }
        if(this.subsystems.getEveryBotPickerSubsystem() != null) {
            this.subsystems.getEveryBotPickerSubsystem().setPickerRelativeSpeed(0.0);
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