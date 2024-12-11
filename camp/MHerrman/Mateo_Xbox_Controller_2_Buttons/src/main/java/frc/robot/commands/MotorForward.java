package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;

/**
 * Our MotorForward class. It extends Command.
 */
public class MotorForward extends Command{
    private FeederSubsystem feeder; // Lets us use our subsystem's feeder for this command.

    /**
     * Our initializer.
     * @param feeder
     */
    public MotorForward(FeederSubsystem feeder){
        this.feeder = feeder; // Initializes this instance of feeder to feeder.
        addRequirements(feeder); // Adds feeder as a requirement.
    }

    /**
     * Our executor.
     */
    @Override
    public void execute(){
    // Our executor. Executes all of our commands and code that is needed.
        feeder.setForwardSpeed(); // Calls on the setForwardSpeed command in the feeder subsystem.
    }

    /**
     * Our isFinished.
     */
    @Override
    public boolean isFinished(){
        return false;
    }
}
