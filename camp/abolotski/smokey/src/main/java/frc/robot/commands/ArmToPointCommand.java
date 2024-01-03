// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: ArmToPointCommand.java
// Intent: Forms a command to move the dual part arm to a y and z point in space.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

/**
 * Class to form a command to move the dual part arm to a y and z point in space.
 */
public class ArmToPointCommand extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private double yValue;
    private double zValue;
    private boolean done = false;

    /**
     * Constructor to drive the arm to a specific point in space
     * @param theArmSubsystem - the arm subsystem
     * @param yPointMeters - the horizontal 'y point' measured from the extension arm pivot point to a normal from the end of the extension arm
     * @param zPointMeters - the vertical 'z point' measured from the floor to the end of the extension arm
     */
    public ArmToPointCommand(ArmSubsystem theArmSubsystem,
                             double yPointMeters,
                             double zPointMeters) {
        this.armSubsystem = theArmSubsystem;
        this.yValue = yPointMeters;
        this.zValue = zPointMeters;
        addRequirements(this.armSubsystem);
    }

    /**
     * Protected method to allow sub classes to update the y position in meters
     * @param armPositionInMetersY - y position in meters
     */
    protected void setYValue(double armPositionInMetersY) {
        yValue = armPositionInMetersY;
    }    

    /**
     * Protected method to allow sub classes to update the z position in meters
     * @param armPositionInMetersZ - z position in meters
     */
    protected void setZValue(double armPositionInMetersZ) {
        zValue = armPositionInMetersZ;
    }

    @Override
    public void initialize() {
        // when the point in space is invalid just be done
        done = (this.armSubsystem.setArmToPointInSpace(yValue, zValue) == false);
        if(done)
        {
            System.out.println("The requested arm point is invalid for the current arms!!! Y = " + yValue + " Z = " + zValue);
        }
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