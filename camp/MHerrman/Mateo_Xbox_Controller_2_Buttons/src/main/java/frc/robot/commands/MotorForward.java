package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;

public class MotorForward extends Command{
    private FeederSubsystem feeder;
    public MotorForward(FeederSubsystem feeder){
        feeder = new FeederSubsystem();
    }

    public void initialize(){

    }

    public void execute(){
        feeder.setForwardSpeed();
    }

    public boolean isFinished(){
        return false;
    }
}