
// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: ArmToExtensionsCommand.java
// Intent: Forms a command to move the dual part arm to a y and z point in space.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

/**
 * Class to form a command to move the dual part arm to specific extension values.
 */
public class ArmToExtensionsCommand extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private double horizontalExtensionValue;
    private double verticalExtensionValue;
    private boolean done = false;

    /**
     * Constructor to drive the arm to a specific tuple of extension values
     * @param theArmSubsystem - the arm subsystem
     * @param horizontalExtensionMeters - the horizontal arm extension in meters
     * @param verticalExtensionMeters - the vertical arm extension in meters
     */
    public ArmToExtensionsCommand(ArmSubsystem theArmSubsystem,
                             double horizontalExtensionMeters,
                             double verticalExtensionMeters) {
        this.armSubsystem = theArmSubsystem;
        this.horizontalExtensionValue = horizontalExtensionMeters;
        this.verticalExtensionValue = verticalExtensionMeters;
        addRequirements(this.armSubsystem);
    }

    /**
     * Protected method to allow sub classes to update the y position in meters
     * @param armPositionInMetersY - y position in meters
     */
    protected void setHorizontalExtension(double horizontalArmExtensionInMeters) {
        horizontalExtensionValue = horizontalArmExtensionInMeters;
    }    

    /**
     * Protected method to allow sub classes to update the z position in meters
     * @param armPositionInMetersZ - z position in meters
     */
    protected void setVerticalExtension(double verticalArmExtensionInMeters) {
        verticalExtensionValue = verticalArmExtensionInMeters;
    }

    @Override
    public void initialize() {
        done = false;
        this.armSubsystem.setArmExtensions(horizontalExtensionValue, verticalExtensionValue);
    }

    @Override
    public void execute() {
        if(!done) {
            done = this.armSubsystem.isRequestedArmMovementComplete();
        }
    }

    @Override
    public void end(boolean interrupted) {
        if(interrupted){
            done = true;
        }
        this.armSubsystem.setArmSpeeds(0.0, 0.0);
    }

    @Override
    public boolean isFinished(){
        return done;
    }    
}