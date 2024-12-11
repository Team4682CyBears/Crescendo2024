package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;

public class MotorStop extends Command{
// Our motorstop class. It extends Command.
    private FeederSubsystem feeder; // Lets us use our subsystem's feeder for this command.

    public MotorStop(FeederSubsystem feeder){
    // Our initializer. Initializes all the variables we need, and adds any requirements we need.
        this.feeder = feeder; // Initializes this instance of feeder to feeder.
        addRequirements(feeder); // Adds feeder as a requirement.
    }

    public void execute(){
    // Our executor. Executes all of our commands and code that is needed.
        feeder.setAllStop(); // Calls on the setAllStop command in our feeder subsystem.
    }

    public boolean isFinished(){
        return false;
    }
}
