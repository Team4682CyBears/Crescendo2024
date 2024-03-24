// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: DrivetrainAccelerationSubsystem.java
// Intent: Forms a subsystem to control ordered access to writeable variables in DrivetrainSubsystem without requring access there.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.MotorUtils;

public class DrivetrainAccelerationSubsystem extends SubsystemBase {

    private DrivetrainSubsystem currentDrivetrainSubsystem = null;

    private final double maximumReductionFactor = 1.0;
    private final double defaultReductionFactor = 1.0;
    private double reductionFactor = defaultReductionFactor;
    private double reductionFactorIncrement = 0.1;

    /**
     * Subsystem that will help coordinate access to the DrivetrainSubsystem
     * 
     * @param currentDrivetrain -
     */
    public DrivetrainAccelerationSubsystem(DrivetrainSubsystem currentDrivetrain) {
        currentDrivetrainSubsystem = currentDrivetrain;
        this.reductionFactor = currentDrivetrainSubsystem.getAccelerationReductionFactor();
    }

    /**
     * Method to decrement the power reduction factor
     */
    public void decrementReductionFactor() {
        reductionFactor = MotorUtils.truncateValue(
                reductionFactor - reductionFactorIncrement,
                reductionFactorIncrement,
                maximumReductionFactor);
        this.updateReductionFactor();
    }

    /**
     * Method to increment the power reduction factor
     */
    public void incrementReductionFactor() {
        reductionFactor = MotorUtils.truncateValue(
                reductionFactor + reductionFactorIncrement,
                reductionFactorIncrement,
                maximumReductionFactor);
        this.updateReductionFactor();
    }

    /**
     * Method to reset the power reduction factor
     */
    public void resetPowerReductionFactor() {
        reductionFactor = defaultReductionFactor;
        this.updateReductionFactor();
    }

    @Override
    public void periodic() {
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation
    }

    public void updateReductionFactor() {
        if (this.currentDrivetrainSubsystem != null) {
            this.currentDrivetrainSubsystem.setAccelerationReductionFactor(this.reductionFactor);
        }
    }
}
