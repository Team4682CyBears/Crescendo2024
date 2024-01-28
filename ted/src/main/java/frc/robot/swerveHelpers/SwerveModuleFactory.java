// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: SwerveModuleFactory.java
// Intent: SwerveModuleFactory class ... a modified copy of SWS content.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.swerveHelpers;

import com.swervedrivespecialties.swervelib.ModuleConfiguration;
import frc.robot.swerveHelpers.SteerController;

import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public class SwerveModuleFactory<DriveConfiguration, SteerConfiguration> {
    private final ModuleConfiguration moduleConfiguration;
    private final DriveControllerFactory<?, DriveConfiguration> driveControllerFactory;
    private final SteerControllerFactory<?, SteerConfiguration> steerControllerFactory;

    public SwerveModuleFactory(ModuleConfiguration moduleConfiguration,
                               DriveControllerFactory<?, DriveConfiguration> driveControllerFactory,
                               SteerControllerFactory<?, SteerConfiguration> steerControllerFactory) {
        this.moduleConfiguration = moduleConfiguration;
        this.driveControllerFactory = driveControllerFactory;
        this.steerControllerFactory = steerControllerFactory;
    }

    public SwerveModule create(DriveConfiguration driveConfiguration, SteerConfiguration steerConfiguration) {
        var driveController = driveControllerFactory.create(driveConfiguration, moduleConfiguration);
        var steerController = steerControllerFactory.create(steerConfiguration, moduleConfiguration);

        return new ModuleImplementation(driveController, steerController);
    }

    public SwerveModule create(ShuffleboardLayout container, DriveConfiguration driveConfiguration, SteerConfiguration steerConfiguration) {
        var driveController = driveControllerFactory.create(
                container,
                driveConfiguration,
                moduleConfiguration
        );
        var steerContainer = steerControllerFactory.create(
                container,
                steerConfiguration,
                moduleConfiguration
        );

        return new ModuleImplementation(driveController, steerContainer);
    }

    private static class ModuleImplementation implements SwerveModule {
        private final DriveController driveController;
        private final SteerController steerController;

        private ModuleImplementation(DriveController driveController, SteerController steerController) {
            this.driveController = driveController;
            this.steerController = steerController;
        }

        @Override
        public double getAbsoluteEncoderOffset(){
            return steerController.getAbsoluteEncoderOffset();
        }

        @Override
        public double getDriveVelocity() {
            return driveController.getStateVelocity();
        }

        @Override
        public double getDriveDistance() {
            return driveController.getDistance();
        }

        @Override
        public void setAbsoluteEncoderOffset(){
            steerController.setAbsoluteEncoderOffset();
        }

        @Override
        public void setDriveDistance(double distance) {
            driveController.setDistance(distance);;
        }

        @Override
        public double getSteerAngle() {
            return steerController.getStateAngle();
        }

        @Override
        public void set(double driveVoltage, double steerAngle) {
            steerAngle %= (2.0 * Math.PI);
            if (steerAngle < 0.0) {
                steerAngle += 2.0 * Math.PI;
            }

            double difference = steerAngle - getSteerAngle();
            // Change the target angle so the difference is in the range [-pi, pi) instead of [0, 2pi)
            if (difference >= Math.PI) {
                steerAngle -= 2.0 * Math.PI;
            } else if (difference < -Math.PI) {
                steerAngle += 2.0 * Math.PI;
            }
            difference = steerAngle - getSteerAngle(); // Recalculate difference

            // If the difference is greater than 90 deg or less than -90 deg the drive can be inverted so the total
            // movement of the module is less than 90 deg
            if (difference > Math.PI / 2.0 || difference < -Math.PI / 2.0) {
                // Only need to add 180 deg here because the target angle will be put back into the range [0, 2pi)
                steerAngle += Math.PI;
                driveVoltage *= -1.0;
            }

            // Put the target angle back into the range [0, 2pi)
            steerAngle %= (2.0 * Math.PI);
            if (steerAngle < 0.0) {
                steerAngle += 2.0 * Math.PI;
            }

            driveController.setReferenceVoltage(driveVoltage);
            steerController.setReferenceAngle(steerAngle);
        }
    }
}