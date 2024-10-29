package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.*;

import edu.wpi.first.wpilibj2.command.Subsystem;

/**
 * Represents a motor subsystem that controls a single motor.
 */
public class MotorSubsystem implements Subsystem {

    private CANSparkMax motor;

    /**
     * Constructs a MotorSubsystem object with the specified motor port.
     * 
     * @param motorPort the port number of the motor
     */
    public MotorSubsystem(int motorPort) {
        this.motor = new CANSparkMax(motorPort, MotorType.kBrushless);
    }

    /**
     * Clamps a value between a minimum and maximum range.
     *
     * @param value The value to be clamped.
     * @param min   The minimum value of the range.
     * @param max   The maximum value of the range.
     * @return The clamped value.
     */
    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /**
     * Sets the speed of the motor.
     * 
     * @param speed the desired speed of the motor (-1.0 to 1.0)
     */
    public void setSpeed(double speed) {
        motor.set(clamp(speed, -1.0, 1.0));
    }

    /**
     * Gets the current speed of the motor.
     * 
     * @return the current speed of the motor
     */
    public double getSpeed() {
        return motor.get();
    }
}
