package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;

public class MotorBackward extends Command{
    private FeederSubsystem feeder;

    public MotorBackward(FeederSubsystem feeder){
        this.feeder = feeder;
        addRequirements(feeder);
    }

    public void initialize(){

    }

    public void execute(){
        feeder.setBackwardSpeed();
    }

    public boolean isFinished(){
        return false;
    }
}
