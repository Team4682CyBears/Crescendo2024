// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: SwerveTrajectoryConfig.java
// Intent: Forms a class that holds config for swerve trajectories.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.common;

import edu.wpi.first.math.trajectory.TrajectoryConfig;

/**
 * A class that extends TrajectoryConfig to also hold rotational max velocity and acceleration
 */
public class SwerveTrajectoryConfig extends TrajectoryConfig {
    double maxRotationalVelocity;
    double maxRotationalAcceleration;

    /**
     * Constructor for SwerveTrajectoryConfig
     * @param maxVelocityMetersPerSecond
     * @param maxAccelerationMetersPerSecondSq
     * @param maxRotationalVelocityRadiansPerSecond
     * @param maxRotationalAccelerationRadiansPerSecondSq
     */
    public SwerveTrajectoryConfig(
        double maxVelocityMetersPerSecond,
        double maxAccelerationMetersPerSecondSq,
        double maxRotationalVelocityRadiansPerSecond,
        double maxRotationalAccelerationRadiansPerSecondSq){
            super(maxVelocityMetersPerSecond, maxAccelerationMetersPerSecondSq);
            this.maxRotationalVelocity = maxRotationalVelocityRadiansPerSecond;
            this.maxRotationalAcceleration = maxRotationalAccelerationRadiansPerSecondSq;
        }

    /**
     * a method to return the max rotational velocity
     * @return
     */
    public double getMaxRotationalVelocity() {
        return maxRotationalVelocity;
    }

    /**
     * A method to return the rotational acceleration
     * @return
     */
    public double getMaxRotationalAcceleration() {
        return maxRotationalAcceleration;
    }

    /**
     * A method to set the max rotational velocity.
     * @param maxRotationalVelocity
     */
    public void setMaxRotationalVelocity(double maxRotationalVelocity) {
        this.maxRotationalVelocity = maxRotationalVelocity;
    }

    /**
     * A method to set the max rotational acceleration
     * @param maxRotationalAcceleration
     */
    public void setMaxRotationalAcceleration(double maxRotationalAcceleration) {
        this.maxRotationalAcceleration = maxRotationalAcceleration;
    }
}
