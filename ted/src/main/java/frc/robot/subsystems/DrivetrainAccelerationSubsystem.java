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

/**
 * Forms a class to control access to acceleration in DrivetrainSubsystem
 */
public class DrivetrainAccelerationSubsystem extends SubsystemBase {

    private DrivetrainSubsystem currentDrivetrainSubsystem = null;

    private final double maximumReductionFactor = 1.0;
    private final double defaultReductionFactor = 1.0;
    private double reductionFactor = defaultReductionFactor;
    private final double reducedReductionFactor = 0.5; // used for brownout conditions
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
     * Called once per scheduler tick for this subsystem
     */
    @Override
    public void periodic() {
    }

    /**
     * Method to reset the power reduction factor
     */
    public void resetPowerReductionFactor() {
        reductionFactor = defaultReductionFactor;
        this.updateReductionFactor();
    }

    /**
     * Method to set the reduced power reduction factor
     */
    public void setReducedReductionFactor() {
        reductionFactor = reducedReductionFactor;
        this.updateReductionFactor();
    }

    /**
     * Method to toggle the reduced power reduction factor
     * if the reduction factor is not the reducedReductionFactor, set it to the reducedReductionFactor
     * Otherwise, set it to the defaultReductionFactor
     */
    public void togglePowerReductionFactor() {
        if (reductionFactor != reducedReductionFactor){
            reductionFactor = reducedReductionFactor;
        }
        else {
            reductionFactor = defaultReductionFactor;
        }
        this.updateReductionFactor();
    }

    /**
     * Set the reduction speed on the drivetrainSusbystem
     */
    public void updateReductionFactor() {
        if (this.currentDrivetrainSubsystem != null) {
            this.currentDrivetrainSubsystem.setAccelerationReductionFactor(this.reductionFactor);
        }
    }
}
