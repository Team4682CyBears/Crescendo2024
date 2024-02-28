package frc.robot.swerveHelpers;

import com.ctre.phoenix.ErrorCode;

public interface AbsoluteEncoder {
    /**
     * Gets the current angle reading of the encoder in radians.
     *
     * @return The current angle in radians. Range: [0, 2pi)
     */
    double getAbsoluteAngle();
        
    /**
     * Gets the last error code from getAbsoluteAngle
     * @return ErrorCode
     */
    ErrorCode getLastError();
}