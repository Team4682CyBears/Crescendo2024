// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: SwerveModule.java
// Intent: SwerveModule class ... a modified copy of SWS content.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.swerveHelpers;

public interface SwerveModule {
    double getAbsoluteEncoderOffset();

    double getDriveVelocity();

    double getDriveDistance();
    void setDriveDistance(double value);

    double getSteerAngle();

    void set(double driveVoltage, double steerAngle);

    void setAbsoluteEncoderOffset();
}