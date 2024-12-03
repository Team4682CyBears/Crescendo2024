package frc.robot.commands;
import java.util.function.DoubleSupplier;

import frc.robot.subsystems.TalonMotorSubsystem;


import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ChangeSpeed extends Command{
    private DoubleSupplier controllerValue;
    private TalonMotorSubsystem useMotor;
    public ChangeSpeed(TalonMotorSubsystem motor, DoubleSupplier xboxLeftX){
        this.useMotor = motor;
        this.addRequirements(useMotor);
        this.controllerValue = xboxLeftX;
    }

    @Override
    public void initialize() {
        
    }

    @Override
    public void execute(){
        this.useMotor.setMotorSpeed(this.controllerValue.getAsDouble());
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
