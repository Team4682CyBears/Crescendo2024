// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: CanCoderFactoryBuilder.java
// Intent: Factory for can coder ... a modified copy of SWS content.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.swerveHelpers;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoder;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.swervedrivespecialties.swervelib.ctre.CanCoderAbsoluteConfiguration;
import com.swervedrivespecialties.swervelib.ctre.CtreUtils;

public class CanCoderFactoryBuilder {
    private Direction direction = Direction.COUNTER_CLOCKWISE;
    private int periodMilliseconds = 10;

    public CanCoderFactoryBuilder withReadingUpdatePeriod(int periodMilliseconds) {
        this.periodMilliseconds = periodMilliseconds;
        return this;
    }

    public CanCoderFactoryBuilder withDirection(Direction direction) {
        this.direction = direction;
        return this;
    }

    public AbsoluteEncoderFactory<CanCoderAbsoluteConfiguration> build() {
        return configuration -> {
            CANCoderConfiguration config = new CANCoderConfiguration();
            config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;
            config.magnetOffsetDegrees = Math.toDegrees(configuration.getOffset());
            config.sensorDirection = direction == Direction.CLOCKWISE;
            config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
            // fix wheel jump issues caused by CANcoder turning opposite direction of motor
            config.sensorDirection = false;

            CANCoder encoder = new CANCoder(configuration.getId());
            CtreUtils.checkCtreError(encoder.configAllSettings(config, 250), "Failed to configure CANCoder");

            CtreUtils.checkCtreError(encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, periodMilliseconds, 250), "Failed to configure CANCoder update rate");

            return new EncoderImplementation(encoder, config.magnetOffsetDegrees);
        };
    }

    private static class EncoderImplementation implements AbsoluteEncoder {
        private final CANCoder encoder;
        private final double offsetDegrees;
        // start out with a general error that is cleared upon first successful reading
        private ErrorCode encoderStatus = ErrorCode.GENERAL_ERROR; 

        private EncoderImplementation(CANCoder encoder, double offsetDegrees) {
            this.encoder = encoder;
            this.offsetDegrees = offsetDegrees;
        }

        @Override
        public double getAbsoluteAngle() {
            double angle = Math.toRadians(encoder.getAbsolutePosition());
            // check error condition in case getAbsolutePosition failed
            // This will be non-zero (zero is ErrorCode.OK) if the frame was not received.
            // https://www.chiefdelphi.com/t/official-sds-mk3-mk4-code/397109/99
            encoderStatus = encoder.getLastError();
            if(encoderStatus != ErrorCode.OK){
                System.out.println("ERROR: Reading absolute encoder position failed.");
            }
            angle %= 2.0 * Math.PI;
            if (angle < 0.0) {
                angle += 2.0 * Math.PI;
            }

            return angle;
        }

        @Override
        public ErrorCode getLastError(){
            return encoderStatus;
        }

        @Override
        public double getOffset(){
            return encoder.configGetMagnetOffset(250);
        }

        @Override
        public void setOffset(){
            CtreUtils.checkCtreError(encoder.configMagnetOffset(offsetDegrees, 250), "Failed to configure CANCoder Offset!");
        }
    }

    public enum Direction {
        CLOCKWISE,
        COUNTER_CLOCKWISE
    }
}