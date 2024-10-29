package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.*;

import edu.wpi.first.wpilibj2.command.Subsystem;

public class MotorSubsystem implements Subsystem {

    private CANSparkMax motor;

    public MotorSubsystem(int motorPort) {
        this.motor = new CANSparkMax(motorPort, MotorType.kBrushless);
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    public void setSpeed(double speed) {
        motor.set(clamp(speed, -1.0, 1.0));
    }

    public double getSpeed() {
        return motor.get();
    }
}
