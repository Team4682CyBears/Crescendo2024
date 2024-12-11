package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.MotorTalon;

public class moveMotor extends Command{
    MotorTalon motorTalonToMove;
    BooleanSupplier rightBumperSupplier;
    BooleanSupplier leftBumperSupplier;


    public moveMotor(MotorTalon motorTalon, BooleanSupplier booleanXboxControllerSupplierLeft, BooleanSupplier booleanXboxControllerSupplierRight){
        addRequirements(motorTalon);
        //save constructor inputs to class variables
        motorTalonToMove = motorTalon;
        leftBumperSupplier = booleanXboxControllerSupplierLeft;
        rightBumperSupplier = booleanXboxControllerSupplierRight;
    }

    @Override
    public void execute(){
        //When both bumpers are pressed, the left bumper is used.
        if(leftBumperSupplier.getAsBoolean()){
            motorTalonToMove.motorReverse();
        } else if(rightBumperSupplier.getAsBoolean()){
            motorTalonToMove.motorForwards();
        } else {
            //no bumper pressed stop the motor
            motorTalonToMove.motorStop();
        }
    }

    @Override
    public boolean isFinished(){
        return false;
    }
}
