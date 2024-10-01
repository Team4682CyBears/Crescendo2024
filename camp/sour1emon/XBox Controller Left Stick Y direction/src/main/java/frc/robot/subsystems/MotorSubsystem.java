package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class MotorSubsystem implements Subsystem {

    private PWMSparkMax motor;

    public MotorSubsystem(int motorPort) {
        motor = new PWMSparkMax(motorPort);
        motor.set(0.0);
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
