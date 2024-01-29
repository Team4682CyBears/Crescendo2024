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
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.swerveHelpers.AbsoluteEncoder;
import frc.robot.swerveHelpers.AbsoluteEncoderFactory;

import frc.robot.swerveLib.ctre.CanCoderAbsoluteConfiguration;
import frc.robot.swerveLib.ctre.CtreUtils;

public class CanCoderFactoryBuilder {
    private Direction direction = Direction.COUNTER_CLOCKWISE;

    public CanCoderFactoryBuilder withDirection(Direction direction) {
        this.direction = direction;
        return this;
    }

    public AbsoluteEncoderFactory<CanCoderAbsoluteConfiguration> build() {
        return configuration -> {
             CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
            canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
            canCoderConfig.MagnetSensor.MagnetOffset = CtreUtils.convertFromRadiansToNormalizedDecmil(configuration.getOffset());
            // seems there is no equivalent for can coder config in v6
// WAS           config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
            // fix wheel jump issues caused by CANcoder turning opposite direction of motor
// WAS:            config.sensorDirection = false;
            canCoderConfig.MagnetSensor.SensorDirection = (direction == Direction.CLOCKWISE ? SensorDirectionValue.Clockwise_Positive : SensorDirectionValue.CounterClockwise_Positive);

            CANcoder encoder = new CANcoder(configuration.getId());
            CtreUtils.checkCtreError(encoder.getConfigurator().apply(canCoderConfig, 250), "Failed to configure CANCoder");
            
            return new EncoderImplementation(encoder, Math.toDegrees(configuration.getOffset()));
        };
    }

    private static class EncoderImplementation implements AbsoluteEncoder {
        private final CANcoder encoder;
        private final double offsetDegrees;
        // start out with a general error that is cleared upon first successful reading
        private StatusCode encoderStatus = StatusCode.GeneralError; 

        private EncoderImplementation(CANcoder encoder, double offsetDegrees) {
            this.encoder = encoder;
            this.offsetDegrees = offsetDegrees;
        }

        @Override
        public double getAbsoluteAngle() {
            double angle = Math.toRadians(encoder.getAbsolutePosition().getValueAsDouble());
            // check error condition in case getAbsolutePosition failed
            // This will be non-zero (zero is ErrorCode.OK) if the frame was not received.
            // https://www.chiefdelphi.com/t/official-sds-mk3-mk4-code/397109/99
            encoderStatus = encoder.getFaultField().getStatus();
            if(encoderStatus != StatusCode.OK){
                System.out.println("ERROR: Reading absolute encoder position failed.");
            }
            angle %= 2.0 * Math.PI;
            if (angle < 0.0) {
                angle += 2.0 * Math.PI;
            }

            return angle;
        }

        @Override
        public StatusCode getLastError(){
            return encoderStatus;
        }

        @Override
        public double getOffset(){
            return encoder.getPosition().getValueAsDouble();
        }

        @Override
        public void setOffset(){
            CtreUtils.checkCtreError(encoder.setPosition(offsetDegrees, 250), "Failed to configure CANCoder Offset!");
        }
    }

    public enum Direction {
        CLOCKWISE,
        COUNTER_CLOCKWISE
    }
}