// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: SwerveModuleHelper.java
// Intent: Same name extension files based on Swerve Drive Specalties codebase but also ported from phoenix5 to phoenix6
// SDS codebase found at: https://github.com/SwerveDriveSpecialties/Do-not-use-swerve-lib-2022-unmaintained/tree/develop/src/main/java/com/swervedrivespecialties/swervelib
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.swerveHelpers;

import frc.robot.swerveLib.ModuleConfiguration;
import frc.robot.swerveLib.ctre.CanCoderAbsoluteConfiguration;
import frc.robot.swerveLib.ctre.Falcon500SteerConfiguration;

import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;

public final class SwerveModuleHelper {
    private SwerveModuleHelper() {
    }

    private static DriveControllerFactory<?, Integer> getFalcon500DriveFactory(SwerveModuleConfiguration configuration) {
        return new Falcon500DriveControllerFactoryBuilder()
                .withVoltageCompensation(configuration.getNominalVoltage())
                .withCurrentLimit(configuration.getDriveCurrentLimit())
                .withStatorCurrentLimit(configuration.getDriveStatorCurrentLimit())
                .withSupplyVoltageTimeConstant(configuration.getDriveSupplyVoltageTimeConstant())
                .build();
    }

    private static SteerControllerFactory<?, Falcon500SteerConfiguration<CanCoderAbsoluteConfiguration>> getFalcon500SteerFactory(SwerveModuleConfiguration configuration) {
        return new Falcon500SteerControllerFactoryBuilder()
                .withVoltageCompensation(configuration.getNominalVoltage())
                .withPidConstants(0.2, 0.0, 0.1)
                .withCurrentLimit(configuration.getSteerCurrentLimit())
                .build(new CanCoderFactoryBuilder()
                        .build());
    }

    /**
     * Creates a Wcp swerve module that uses Falcon 500s for driving and steering.
     * Module information is displayed in the specified ShuffleBoard container.
     *
     * @param container        The container to display module information in.
     * @param configuration    Module configuration parameters to use.
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive Falcon 500.
     * @param steerMotorPort   The CAN ID of the steer Falcon 500.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @return The configured swerve module.
     */
    public static SwerveModule createFalcon500(
            ShuffleboardLayout container,
            SwerveModuleConfiguration configuration,
            ModuleConfiguration gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double steerOffset
    ) {
        return new SwerveModuleFactory<>(
                gearRatio,
                getFalcon500DriveFactory(configuration),
                getFalcon500SteerFactory(configuration)
        ).create(
                container,
                driveMotorPort,
                new Falcon500SteerConfiguration<>(
                        steerMotorPort,
                        new CanCoderAbsoluteConfiguration(steerEncoderPort, steerOffset)
                )
        );
    }

    /**
     * Creates a Wcp swerve module that uses Falcon 500s for driving and steering.
     * Module information is displayed in the specified ShuffleBoard container.
     *
     * @param container        The container to display module information in.
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive Falcon 500.
     * @param steerMotorPort   The CAN ID of the steer Falcon 500.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @return The configured swerve module.
     */
    public static SwerveModule createFalcon500(
            ShuffleboardLayout container,
            ModuleConfiguration gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double steerOffset
    ) {
        return createFalcon500(container, new SwerveModuleConfiguration(), gearRatio, driveMotorPort, steerMotorPort, steerEncoderPort, steerOffset);
    }

    /**
     * Creates a Wcp swerve module that uses Falcon 500s for driving and steering.
     *
     * @param configuration    Module configuration parameters to use.
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive Falcon 500.
     * @param steerMotorPort   The CAN ID of the steer Falcon 500.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @return The configured swerve module.
     */
    public static SwerveModule createFalcon500(
            SwerveModuleConfiguration configuration,
            ModuleConfiguration gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double steerOffset
    ) {
        return new SwerveModuleFactory<>(
                gearRatio,
                getFalcon500DriveFactory(configuration),
                getFalcon500SteerFactory(configuration)
        ).create(
                driveMotorPort,
                new Falcon500SteerConfiguration<>(
                        steerMotorPort,
                        new CanCoderAbsoluteConfiguration(steerEncoderPort, steerOffset)
                )
        );
    }

    /**
     * Creates a Wcp swerve module that uses Falcon 500s for driving and steering.
     *
     * @param gearRatio        The gearing configuration the module is in.
     * @param driveMotorPort   The CAN ID of the drive Falcon 500.
     * @param steerMotorPort   The CAN ID of the steer Falcon 500.
     * @param steerEncoderPort The CAN ID of the steer CANCoder.
     * @param steerOffset      The offset of the CANCoder in radians.
     * @return The configured swerve module.
     */
    public static SwerveModule createFalcon500(
            ModuleConfiguration gearRatio,
            int driveMotorPort,
            int steerMotorPort,
            int steerEncoderPort,
            double steerOffset
    ) {
        return createFalcon500(new SwerveModuleConfiguration(), gearRatio, driveMotorPort, steerMotorPort, steerEncoderPort, steerOffset);
    }

}
    

