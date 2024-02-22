// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: .java
// Intent: Same name file port of Swerve Drive Specalties codebase from phoenix5 
// to phoenix6
// SDS codebase found at: https://github.com/SwerveDriveSpecialties/Do-not-use-swerve-lib-2022-unmaintained/tree/develop/src/main/java/com/swervedrivespecialties/swervelib
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.swerveLib.ctre;

public class CanCoderAbsoluteConfiguration {
    private final int id;
    private final double offset;

    public CanCoderAbsoluteConfiguration(int id, double offset) {
        this.id = id;
        this.offset = offset;
    }

    public int getId() {
        return id;
    }

    public double getOffset() {
        return offset;
    }
}
