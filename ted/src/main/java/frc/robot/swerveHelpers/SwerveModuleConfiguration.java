// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: SwerveModuleConfiguration.java
// Intent: Same name extension files based on Swerve Drive Specalties codebase but also ported from phoenix5 to phoenix6
// SDS codebase found at: https://github.com/SwerveDriveSpecialties/Do-not-use-swerve-lib-2022-unmaintained/tree/develop/src/main/java/com/swervedrivespecialties/swervelib
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.swerveHelpers;

import java.util.Objects;

/**
 * Additional Swerve module configuration parameters.
 * <p>
 * The configuration parameters here are used to customize the behavior of the Swerve swerve module.
 * Each setting is initialized to a default that should be adequate for most use cases.
 */
public class SwerveModuleConfiguration {
    private double nominalVoltage = 12.0;
    private double driveCurrentLimit = 50.0;
    private double steerCurrentLimit = 20.0;
    
    // Added in Glacier Peak 2024 per converstion with Jack mentor Stephanie and Squirrels student
    private double driveStatorCurrentLimit = 100.0;

    // Added in Glacier Peak 2024 per converstion with Jack mentor Stephanie 
    // 20ms voltage ramp is consistent with legacy falcons. Krakens default of 0
    // causes high current spike on motor
    private double driveSupplyVoltageTimeConstant = 0.02;

    public double getNominalVoltage() {
        return nominalVoltage;
    }

    public void setNominalVoltage(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
    }

    public double getDriveCurrentLimit() {
        return driveCurrentLimit;
    }

    public void setDriveCurrentLimit(double driveCurrentLimit) {
        this.driveCurrentLimit = driveCurrentLimit;
    }

    public double getDriveStatorCurrentLimit() {
        return this.driveStatorCurrentLimit;
    }

    public void setDriveStatorCurrentLimit(double driveStatorCurrentLimit) {
        this.driveStatorCurrentLimit = driveStatorCurrentLimit;
    }

    public double getDriveSupplyVoltageTimeConstant() {
        return this.driveSupplyVoltageTimeConstant;
    }

    public void setDriveSupplyVoltageTimeConstant(double driveSupplyVoltageTimeConstant) {
        this.driveSupplyVoltageTimeConstant = driveSupplyVoltageTimeConstant;
    }

    public double getSteerCurrentLimit() {
        return steerCurrentLimit;
    }

    public void setSteerCurrentLimit(double steerCurrentLimit) {
        this.steerCurrentLimit = steerCurrentLimit;
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        SwerveModuleConfiguration that = (SwerveModuleConfiguration) o;
        return Double.compare(that.getNominalVoltage(), getNominalVoltage()) == 0 && Double.compare(that.getDriveCurrentLimit(), getDriveCurrentLimit()) == 0 && Double.compare(that.getSteerCurrentLimit(), getSteerCurrentLimit()) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(getNominalVoltage(), getDriveCurrentLimit(), getSteerCurrentLimit());
    }

    @Override
    public String toString() {
        return "SwerveModuleConfiguration{" +
                "nominalVoltage=" + nominalVoltage +
                ", driveCurrentLimit=" + driveCurrentLimit +
                ", steerCurrentLimit=" + steerCurrentLimit +
                '}';
    }
}