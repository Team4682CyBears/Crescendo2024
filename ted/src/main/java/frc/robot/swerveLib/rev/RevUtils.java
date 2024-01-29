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

package frc.robot.swerveLib.rev;

import com.revrobotics.REVLibError;
import edu.wpi.first.wpilibj.DriverStation;

public final class RevUtils {
    private RevUtils() {}

    public static void checkNeoError(REVLibError error, String message) {
        if (error != REVLibError.kOk) {
            DriverStation.reportError(String.format("%s: %s", message, error.toString()), false);
        }
    }
}
