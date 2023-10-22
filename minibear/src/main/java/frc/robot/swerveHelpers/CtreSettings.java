// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: CtreSettings.java
// Intent: CtreSettings class ... a modified copy of SWS content.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.swerveHelpers;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.swervedrivespecialties.swervelib.ctre.CanCoderAbsoluteConfiguration;

import frc.robot.Constants;

public class CtreSettings {

    private static CANCoder backLeftCoder = new CANCoder(Constants.BACK_LEFT_MODULE_STEER_ENCODER);
    private static CANCoder backRightCoder = new CANCoder(Constants.BACK_RIGHT_MODULE_STEER_ENCODER);
    private static CANCoder frontLeftCoder = new CANCoder(Constants.FRONT_LEFT_MODULE_STEER_ENCODER);
    private static CANCoder frontRightCoder = new CANCoder(Constants.FRONT_RIGHT_MODULE_STEER_ENCODER);

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

    private static void PrintCanEncoderCurrentSettings(CANCoder cancoder)
    {
        System.out.println("Get side, has angle: " + cancoder.getAbsolutePosition() + " and "
        + "initializationStrategy == " + cancoder.configGetSensorInitializationStrategy(0).toString());
    }
    
    private static void UpdateSingleCanEncoderDefaultSettings(CANCoder cancoder)
    {
        CANCoderConfiguration config = new CANCoderConfiguration();
        SensorInitializationStrategy strat = SensorInitializationStrategy.BootToAbsolutePosition;
        config.initializationStrategy = strat;
        ErrorCode returnVal = cancoder.configAllSettings(config);
        System.out.println("attemtpted set value == " + strat.toString());
        System.out.println("Set side, getDeviceID == " + cancoder.getDeviceID() + " Error Code: " + returnVal);
        CtreSettings.PrintCanEncoderCurrentSettings(cancoder);
    }
}
