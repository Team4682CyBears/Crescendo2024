package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.MotorTalon;

public class moveMotor extends Command{
    MotorTalon motorTalonToMove;
    BooleanSupplier rightBumperSupplier;
    BooleanSupplier leftBumperSupplier;


    public moveMotor(MotorTalon motorTalon, BooleanSupplier booleanXboxControllerSupplierLeft, BooleanSupplier booleanXboxControllerSupplierRight){
        addRequirements(motorTalon);
        motorTalonToMove = motorTalon;
        leftBumperSupplier = booleanXboxControllerSupplierLeft;
        rightBumperSupplier = booleanXboxControllerSupplierRight;
    }

    @Override
    public void execute(){
        if(leftBumperSupplier.getAsBoolean()){
            motorTalonToMove.motorReverse();
        } else if(rightBumperSupplier.getAsBoolean()){
            motorTalonToMove.motorForwards();
        } else {
            motorTalonToMove.motorStop();
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
