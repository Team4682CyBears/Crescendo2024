package frc.robot.commands;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.MotorTalon;

public class moveMotor extends Command{
    //globals here
    MotorTalon motorTalonToMove;
    BooleanSupplier rightBumperSupplier;
    BooleanSupplier leftBumperSupplier;


    public moveMotor(MotorTalon motorTalon, BooleanSupplier booleanXboxControllerSupplierLeft, BooleanSupplier booleanXboxControllerSupplierRight){
        //require motor (or it wont work. Cant spin a motor without a motor)
        addRequirements(motorTalon);
        //construct the globals we set earlier
        motorTalonToMove = motorTalon;
        leftBumperSupplier = booleanXboxControllerSupplierLeft;
        rightBumperSupplier = booleanXboxControllerSupplierRight;
    }

    @Override
    public void execute(){
        //left bumper is more importaint than the right bumper
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
        //its never over
        return false;
    }
}
