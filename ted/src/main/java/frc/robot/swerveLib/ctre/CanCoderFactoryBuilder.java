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

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.swerveLib.AbsoluteEncoder;
import frc.robot.swerveLib.AbsoluteEncoderFactory;

public class CanCoderFactoryBuilder {
    private Direction direction = Direction.COUNTER_CLOCKWISE;

    // based on content at: https://v6.docs.ctr-electronics.com/en/2023-v6/docs/migration/migration-guide/feature-replacements-guide.html#velocity-measurement-period-window
    // it appears that the next set of lines devoted to updating the period is no longer necessary
    // "In Phoenix 6, the velocity rolling average window in Talon FX and CANcoder has been replaced with a Kalman filter, resulting in a less noisy velocity signal with a minimal impact on latency. As a result, the velocity measurement period/window configs are no longer necessary in Phoenix 6 and have been removed."
/* WAS:
    private int periodMilliseconds = 10;

    public CanCoderFactoryBuilder withReadingUpdatePeriod(int periodMilliseconds) {
        this.periodMilliseconds = periodMilliseconds;
        return this;
    }
*/
    public CanCoderFactoryBuilder withDirection(Direction direction) {
        System.out.println("shouldn't be here #0!");
        this.direction = direction;
        return this;
    }

    public AbsoluteEncoderFactory<CanCoderAbsoluteConfiguration> build()  {
        return configuration -> {
            System.out.println("shouldn't be here #1!");
// WAS:            CANCoderConfiguration config = new CANCoderConfiguration();
            CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
// WAS:            config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
            canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1; // seems like implementation discontinuity between libraries to me...
// WAS:            config.magnetOffsetDegrees = Math.toDegrees(configuration.getOffset());
            canCoderConfig.MagnetSensor.MagnetOffset = CtreUtils.convertFromRadiansToNormalizedDecmil(configuration.getOffset());
// WAS:            config.sensorDirection = direction == Direction.CLOCKWISE;
            canCoderConfig.MagnetSensor.SensorDirection = (direction == Direction.CLOCKWISE ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive);

// WAS:            CANCoder encoder = new CANCoder(configuration.getId());
            CANcoder encoder = new CANcoder(configuration.getId());
// WAS:            CtreUtils.checkCtreError(encoder.configAllSettings(config, 250), "Failed to configure CANCoder");
            CtreUtils.checkCtreError(encoder.getConfigurator().apply(canCoderConfig, 250), "Failed to configure CANCoder");
          
            // based on content at: https://v6.docs.ctr-electronics.com/en/2023-v6/docs/migration/migration-guide/feature-replacements-guide.html#velocity-measurement-period-window
            // it appears that the next line is no longer necessary
            // "In Phoenix 6, the velocity rolling average window in Talon FX and CANcoder has been replaced with a Kalman filter, resulting in a less noisy velocity signal with a minimal impact on latency. As a result, the velocity measurement period/window configs are no longer necessary in Phoenix 6 and have been removed."
// WAS:            CtreUtils.checkCtreError(encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, periodMilliseconds, 250), "Failed to configure CANCoder update rate");

            return new EncoderImplementation(encoder);
        };
    }

    private static class EncoderImplementation implements AbsoluteEncoder {
        private final CANcoder encoder;

        private EncoderImplementation(CANcoder encoder) {
            this.encoder = encoder;
            System.out.println("shouldn't be here #2!");
        }

        @Override
        public double getAbsoluteAngle() {
            System.out.println("shouldn't be here #3!");
            double angle = 2.0 * Math.PI * encoder.getAbsolutePosition().getValueAsDouble();
            angle %= 2.0 * Math.PI;
            if (angle < 0.0) {
                angle += 2.0 * Math.PI;
            }

            return angle;
        }
    }

    public enum Direction {
        CLOCKWISE,
        COUNTER_CLOCKWISE
    }

}
