// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: .java
// Intent: Same name extension files based on Swerve Drive Specalties codebase but also ported from phoenix5 to phoenix6
// SDS codebase found at: https://github.com/SwerveDriveSpecialties/Do-not-use-swerve-lib-2022-unmaintained/tree/develop/src/main/java/com/swervedrivespecialties/swervelib
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.swerveHelpers;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;

import frc.robot.swerveLib.ctre.CanCoderAbsoluteConfiguration;

import frc.robot.control.Constants;

public class CtreSettings {

    private static CANcoder backLeftCoder = new CANcoder(Constants.BACK_LEFT_MODULE_STEER_ENCODER);
    private static CANcoder backRightCoder = new CANcoder(Constants.BACK_RIGHT_MODULE_STEER_ENCODER);
    private static CANcoder frontLeftCoder = new CANcoder(Constants.FRONT_LEFT_MODULE_STEER_ENCODER);
    private static CANcoder frontRightCoder = new CANcoder(Constants.FRONT_RIGHT_MODULE_STEER_ENCODER);

    public static void PrintAllCanEncoderCurrentSettings()
    {
        CtreSettings.PrintCanEncoderCurrentSettings(backLeftCoder);
        CtreSettings.PrintCanEncoderCurrentSettings(backRightCoder);
        CtreSettings.PrintCanEncoderCurrentSettings(frontLeftCoder);
        CtreSettings.PrintCanEncoderCurrentSettings(frontRightCoder);
    }

    public static void UpdateCanEncoderDefaultSettings()
    {
        CtreSettings.UpdateSingleCanEncoderDefaultSettings(backLeftCoder);
        CtreSettings.UpdateSingleCanEncoderDefaultSettings(backRightCoder);
        CtreSettings.UpdateSingleCanEncoderDefaultSettings(frontLeftCoder);
        CtreSettings.UpdateSingleCanEncoderDefaultSettings(frontRightCoder);

        PrintAllCanEncoderCurrentSettings();
    }

    private static void PrintCanEncoderCurrentSettings(CANcoder canCoder)
    {
        // NOTE: see: https://v6.docs.ctr-electronics.com/en/2023-v6/docs/migration/migration-guide/feature-replacements-guide.html#sensor-initialization-strategy
        // "The Talon FX and CANcoder sensors are always initialized to their absolute position in Phoenix 6."
        System.out.println("Get side, has angle: " + canCoder.getAbsolutePosition());
    }
    
    private static void UpdateSingleCanEncoderDefaultSettings(CANcoder canCoder)
    {
        CANcoderConfiguration config = new CANcoderConfiguration();
        StatusCode returnVal = canCoder.getConfigurator().apply(config);
        System.out.println("Set side, getDeviceID == " + canCoder.getDeviceID() + " Error Code: " + returnVal);
        CtreSettings.PrintCanEncoderCurrentSettings(canCoder);
    }
}
