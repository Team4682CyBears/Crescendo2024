package frc.robot.swerveLib;

public interface DriveController {
    void setReferenceVoltage(double voltage);

    double getStateVelocity();
}
