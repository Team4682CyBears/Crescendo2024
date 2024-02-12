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

// WAS:import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix6.StatusCode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;

public final class CtreUtils {
    private CtreUtils() {
    }

    public static void checkCtreError(StatusCode errorCode, String message) {
// WAS:        if (errorCode != ErrorCode.OK) {
        if (errorCode != StatusCode.OK) {
            DriverStation.reportError(String.format("%s: %s", message, errorCode.toString()), false);
        }
    }

   /**
     * Seems like we need a function to convert radians into a unitary type scale of 0.0 to 1.0
     * @param radians
     * @return Value of 0 to 1 linear normalized radians
     */
    public static double convertFromRadiansToNormalizedDecmil(double radians) {
        double normalized = (MathUtil.angleModulus(radians) + Math.PI) / 2.0 * Math.PI;
        return normalized;
    }

    /**
     * Seems like we need a function to convert radians into a degress from 0 to 360
     * @param radians
     * @return Value of 0 to 1 linear normalized radians
     */
    public static double convertFromRadiansToNormalizedDegrees(double radians) {
        double normalized = CtreUtils.convertFromRadiansToNormalizedDecmil(radians) * 360.0;
        return normalized;
    }
}
