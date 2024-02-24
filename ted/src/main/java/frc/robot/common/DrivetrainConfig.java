// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo 2024
// File: DrivetrainConfig.java
// Intent: Forms a type to hold drivetrain config
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.common;

import frc.robot.swerveLib.ModuleConfiguration;

/**
 * Forms a type to hold drivetrian config
 */
public class DrivetrainConfig {
    private double trackwidthMeters;
    private double wheelbaseMeters;
    private ModuleConfiguration swerveModuleConfiguration;
    private double frontLeftModuleSteerOffset;
    private double frontRightModuleSteerOffset;
    private double backLeftModuleSteerOffset;
    private double backRightModuleSteerOffset;

    public DrivetrainConfig(
            double trackWidthMeters,
            double wheelbaseMeters,
            ModuleConfiguration swerveModuleConfiguration,
            double frontLeftModuleSteerOffset,
            double frontRightModuleSteerOffset,
            double backLeftModuleSteerOffset,
            double backRightModuleSteerOffset) {
        this.trackwidthMeters = trackWidthMeters;
        this.wheelbaseMeters = wheelbaseMeters;
        this.swerveModuleConfiguration = swerveModuleConfiguration;
        this.frontLeftModuleSteerOffset = frontLeftModuleSteerOffset;
        this.frontRightModuleSteerOffset = frontRightModuleSteerOffset;
        this.backLeftModuleSteerOffset = backLeftModuleSteerOffset;
        this.backRightModuleSteerOffset = backRightModuleSteerOffset;
    }

    public double getBackLeftModuleSteerOffset() {
        return backLeftModuleSteerOffset;
    }

    public double getBackRightModuleSteerOffset() {
        return backRightModuleSteerOffset;
    }

    public double getFrontLeftModuleSteerOffset() {
        return frontLeftModuleSteerOffset;
    }

    public double getFrontRightModuleSteerOffset() {
        return frontRightModuleSteerOffset;
    }

    public ModuleConfiguration getSwerveModuleConfiguration() {
        return swerveModuleConfiguration;
    }

    public double getTrackwidthMeters() {
        return trackwidthMeters;
    }

    public double getWheelbaseMeters() {
        return wheelbaseMeters;
    }

    public void setBackLeftModuleSteerOffset(double backLeftModuleSteerOffset) {
        this.backLeftModuleSteerOffset = backLeftModuleSteerOffset;
    }

    public void setBackRightModuleSteerOffset(double backRightModuleSteerOffset) {
        this.backRightModuleSteerOffset = backRightModuleSteerOffset;
    }

    public void setFrontLeftModuleSteerOffset(double frontLeftModuleSteerOffset) {
        this.frontLeftModuleSteerOffset = frontLeftModuleSteerOffset;
    }

    public void setFrontRightModuleSteerOffset(double frontRightModuleSteerOffset) {
        this.frontRightModuleSteerOffset = frontRightModuleSteerOffset;
    }

    public void setSwerveModuleConfiguration(ModuleConfiguration swerveModuleConfiguration) {
        this.swerveModuleConfiguration = swerveModuleConfiguration;
    }

    public void setTrackwidthMeters(double trackwidthMeters) {
        this.trackwidthMeters = trackwidthMeters;
    }

    public void setWheelbaseMeters(double wheelbaseMeters) {
        this.wheelbaseMeters = wheelbaseMeters;
    }

}
