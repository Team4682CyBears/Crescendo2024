package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Talon;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;

public class MotorTalon extends SubsystemBase{
    Talon motorTalon = new Talon(Constants.OperatorConstants.motorTalonPort);

    public MotorTalon(){
        System.out.print("Motor doing stuff");
    }

    public void motorReverse(){
        motorTalon.set(-1.0);
    }

    public void motorForwards(){
        motorTalon.set(1.0);
    }

    @Override
    public void periodic() {
        System.out.println(motorTalon.get());
        super.periodic();
    }
}
