package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;

/**
 * Our MotorStop class. It extends Command.
 */
public class MotorStop extends Command{
    private FeederSubsystem feeder; // Lets us use our subsystem's feeder for this command.

    /**
     * Our initializer.
     */
    public MotorStop(FeederSubsystem feeder){
        this.feeder = feeder; // Initializes this instance of feeder to feeder.
        addRequirements(feeder); // Adds feeder as a requirement.
    }

    /**
     * Our executor.
     */
    @Override
    public void execute(){
    // Our executor. Executes all of our commands and code that is needed.
        feeder.setAllStop(); // Calls on the setAllStop command in our feeder subsystem.
    }

    /**
     * Our isFinished.
     */
    @Override
    public boolean isFinished(){
        return false;
    }
}
