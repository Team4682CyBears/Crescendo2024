package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;

public class MotorForward extends Command{
// Our MotorForward class. It extends Command
    private FeederSubsystem feeder; // Lets us use our subsystem's feeder for this command.

    public MotorForward(FeederSubsystem feeder){
    // Our initializer. Initializes all the variables we need, and adds any requirements we need.
        this.feeder = feeder; // Initializes this instance of feeder to feeder.
        addRequirements(feeder); // Adds feeder as a requirement.
    }

    public void execute(){
    // Our executor. Executes all of our commands and code that is needed.
        feeder.setForwardSpeed(); // Calls on the setForwardSpeed command in the feeder subsystem.
    }

    public boolean isFinished(){
        return false;
    }
}
