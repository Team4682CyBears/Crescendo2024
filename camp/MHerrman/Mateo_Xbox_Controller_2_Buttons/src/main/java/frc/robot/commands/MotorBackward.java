package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;

public class MotorBackward extends Command{
// Our MotorBackward class. It extends Command.
    private FeederSubsystem feeder; // Lets us use our subsystem's feeder for this command.

    
    public MotorBackward(FeederSubsystem feeder){
    // Our initializer. Initializes all the variables we need, and adds any requirements we need.
        this.feeder = feeder; // Initializes this instance of feeder to feeder.
        addRequirements(feeder); // Adds feeder as a requirement.
    }

    public void execute(){
    // Our executor. Executes all of our commands and code that is needed.
        feeder.setBackwardSpeed();
    }

    public boolean isFinished(){
        return false;
    }
}
