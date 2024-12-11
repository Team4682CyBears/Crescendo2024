package frc.robot.commands;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.FeederSubsystem;

public class MotorStop extends Command{
    private FeederSubsystem feeder;
    public MotorStop(FeederSubsystem feeder){
        feeder = new FeederSubsystem();
    }

    public void initialize(){

    }

    public void execute(){
        feeder.setAllStop();
    }

    public boolean isFinished(){
        return false;
    }
}