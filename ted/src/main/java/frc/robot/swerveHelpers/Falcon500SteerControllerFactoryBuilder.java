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

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.StatusCode;

import frc.robot.swerveLib.*;
import frc.robot.swerveLib.ctre.CtreUtils;
import frc.robot.swerveLib.ctre.Falcon500SteerConfiguration;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardContainer;

public final class Falcon500SteerControllerFactoryBuilder {
    private static final int CAN_TIMEOUT_MS = 250;
    private static final int STATUS_FRAME_GENERAL_PERIOD_MS = 250;

    private static final double TICKS_PER_ROTATION = 2048.0;

    // PID configuration
    private double proportionalConstant = Double.NaN;
    private double integralConstant = Double.NaN;
    private double derivativeConstant = Double.NaN;

    // Motion magic configuration
    private double velocityConstant = Double.NaN;
    private double accelerationConstant = Double.NaN;
    private double staticConstant = Double.NaN;

    private double nominalVoltage = Double.NaN;
    private double currentLimit = Double.NaN;

    private static double absAngleTolRadians = Units.degreesToRadians(5);

    public Falcon500SteerControllerFactoryBuilder withPidConstants(double proportional, double integral, double derivative) {
        this.proportionalConstant = proportional;
        this.integralConstant = integral;
        this.derivativeConstant = derivative;
        return this;
    }

    public boolean hasPidConstants() {
        return Double.isFinite(proportionalConstant) && Double.isFinite(integralConstant) && Double.isFinite(derivativeConstant);
    }

    public Falcon500SteerControllerFactoryBuilder withMotionMagic(double velocityConstant, double accelerationConstant, double staticConstant) {
        this.velocityConstant = velocityConstant;
        this.accelerationConstant = accelerationConstant;
        this.staticConstant = staticConstant;
        return this;
    }

    public boolean hasMotionMagic() {
        return Double.isFinite(velocityConstant) && Double.isFinite(accelerationConstant) && Double.isFinite(staticConstant);
    }

    public Falcon500SteerControllerFactoryBuilder withVoltageCompensation(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    public boolean hasVoltageCompensation() {
        return Double.isFinite(nominalVoltage);
    }

    public Falcon500SteerControllerFactoryBuilder withCurrentLimit(double currentLimit) {
        this.currentLimit = currentLimit;
        return this;
    }

    public boolean hasCurrentLimit() {
        return Double.isFinite(currentLimit);
    }

    public <T> SteerControllerFactory<ControllerImplementation, Falcon500SteerConfiguration<T>> build(AbsoluteEncoderFactory<T> absoluteEncoderFactory) {
        return new FactoryImplementation<>(absoluteEncoderFactory);
    }

    private class FactoryImplementation<T> implements SteerControllerFactory<ControllerImplementation, Falcon500SteerConfiguration<T>> {
        private final AbsoluteEncoderFactory<T> encoderFactory;
        private final PositionVoltage positionVoltage = new PositionVoltage(0);

        private FactoryImplementation(AbsoluteEncoderFactory<T> encoderFactory) {
            this.encoderFactory = encoderFactory;
        }

        @Override
        public void addDashboardEntries(ShuffleboardContainer container, ControllerImplementation controller) {
            SteerControllerFactory.super.addDashboardEntries(container, controller);
            container.addNumber("Absolute Encoder Angle", () -> Math.toDegrees(controller.absoluteEncoder.getAbsoluteAngle())); 
            container.addBoolean("Absolute Encoder Sync Status OK", () -> controller.absoluteEncoder.getLastError() == StatusCode.OK);
        }

        @Override
        public ControllerImplementation create(Falcon500SteerConfiguration<T> steerConfiguration, ModuleConfiguration moduleConfiguration) {

            AbsoluteEncoder absoluteEncoder = encoderFactory.create(steerConfiguration.getEncoderConfiguration());
            final double sensorPositionCoefficient = 2.0 * Math.PI / TICKS_PER_ROTATION * moduleConfiguration.getSteerReduction();
            final double sensorVelocityCoefficient = sensorPositionCoefficient * 10.0;

            TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();

            if (hasPidConstants()) {
                motorConfiguration.Slot0.kP = proportionalConstant;
                motorConfiguration.Slot0.kI = integralConstant;
                motorConfiguration.Slot0.kD = derivativeConstant;
            }
            if (hasMotionMagic()) {
                if (hasVoltageCompensation()) {
                    motorConfiguration.Slot0.kV = (1023.0 * sensorVelocityCoefficient / nominalVoltage) * velocityConstant;
                }
                // TODO: What should be done if no nominal voltage is configured? Use a default voltage?

                // TODO: Make motion magic max voltages configurable or dynamically determine optimal values
                motorConfiguration.MotionMagic.MotionMagicCruiseVelocity = 2.0 / velocityConstant / sensorVelocityCoefficient;
                motorConfiguration.MotionMagic.MotionMagicAcceleration = (8.0 - 2.0) / accelerationConstant / sensorVelocityCoefficient;
            }
            if (hasVoltageCompensation()) {
                motorConfiguration.Voltage.PeakForwardVoltage = nominalVoltage;
                motorConfiguration.Voltage.PeakReverseVoltage = -1.0 * nominalVoltage;
            }
            if (hasCurrentLimit()) {
                motorConfiguration.CurrentLimits.SupplyCurrentLimit = currentLimit;
                motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
            }

            TalonFX motor = new TalonFX(steerConfiguration.getMotorPort());
            CtreUtils.checkCtreError(motor.getConfigurator().apply(motorConfiguration), "Failed to configure Falcon 500 settings");

            motorConfiguration.MotorOutput.Inverted = (moduleConfiguration.isSteerInverted() ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive);
            motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

            Double absAngle = absoluteEncoder.getAbsoluteAngle();
            if (absoluteEncoder.getLastError() == StatusCode.OK) {
                // if we were able to read the absolute encoder, then try to set the sensor position
                CtreUtils.checkCtreError(
                    motor.setPosition(absAngle / sensorPositionCoefficient, CAN_TIMEOUT_MS),
                    "WARNING: Failed to set Falcon 500 encoder position.");
            } else {
                // abs encoder is synced periodically, every 10s.  If reading the sensor fails, wait 10s before enabling the robot. 
                System.out.println("WARNING: Reading absolute encoder position failed. Wait 10s before enabling robot.");
            } 

            // Reduce CAN status frame rates
            CtreUtils.checkCtreError(
                motor.getPosition().setUpdateFrequency(STATUS_FRAME_GENERAL_PERIOD_MS, CAN_TIMEOUT_MS),
                "Failed to configure Falcon status frame period"
            );
            
            // THIS IS EXTRA CODE MIKE IS ADDING - why? because I think it might be needed
            motor.getConfigurator().apply(motorConfiguration);
            motor.setControl(this.positionVoltage);

            return new ControllerImplementation(motor,
                    sensorPositionCoefficient,
                    sensorVelocityCoefficient,
                    positionVoltage,
                    absoluteEncoder);
        }
    }

    private static class ControllerImplementation implements SteerController {
        private static final int ENCODER_RESET_ITERATIONS = 500;
        private static final double ENCODER_RESET_MAX_ANGULAR_VELOCITY = Math.toRadians(0.5);

        private final TalonFX motor;
        private final double motorEncoderPositionCoefficient;
        private final double motorEncoderVelocityCoefficient;
        private final PositionVoltage positionVoltage;
        private final AbsoluteEncoder absoluteEncoder;

        private double referenceAngleRadians = 0.0;

        private double resetIteration = 0;

        private ControllerImplementation(TalonFX motor,
                                         double motorEncoderPositionCoefficient,
                                         double motorEncoderVelocityCoefficient,
                                         PositionVoltage positionVoltage,
                                         AbsoluteEncoder absoluteEncoder) {
            this.motor = motor;
            this.motorEncoderPositionCoefficient = motorEncoderPositionCoefficient;
            this.motorEncoderVelocityCoefficient = motorEncoderVelocityCoefficient;
            this.positionVoltage = positionVoltage;
            this.absoluteEncoder = absoluteEncoder;
        }

        @Override
        public double getAbsoluteEncoderOffset(){
            return absoluteEncoder.getOffset();
        }

        @Override
        public double getReferenceAngle() {
            return referenceAngleRadians;
        }

        @Override
        public void setReferenceAngle(double referenceAngleRadians) {
            double currentAngleRadians = motor.getPosition().getValue() * motorEncoderPositionCoefficient;

            // Reset the NEO's encoder periodically when the module is not rotating.
            // Sometimes (~5% of the time) when we initialize, the absolute encoder isn't fully set up, and we don't
            // end up getting a good reading. If we reset periodically this won't matter anymore.
            if (motor.getVelocity().getValueAsDouble() * motorEncoderVelocityCoefficient < ENCODER_RESET_MAX_ANGULAR_VELOCITY) {
                if (++resetIteration >= ENCODER_RESET_ITERATIONS) {
                    resetIteration = 0;
                    double absoluteAngle = absoluteEncoder.getAbsoluteAngle();
                    if (absoluteEncoder.getLastError() == StatusCode.OK){
                        if (Math.abs(MathUtil.angleModulus(absoluteAngle - currentAngleRadians)) 
                        > absAngleTolRadians){
                            System.out.println("WARNING: Large error encountered when syncing absolute encoder from " + 
                            currentAngleRadians + " to " + absoluteAngle + ".");
                        }
                        motor.setPosition(absoluteAngle / motorEncoderPositionCoefficient);
                        currentAngleRadians = absoluteAngle;    
                    } else {
                        System.out.println("WARNING: Syncing absolute encoder position failed.");
                    }
                }
            } else {
                resetIteration = 0;
            }

            double currentAngleRadiansMod = currentAngleRadians % (2.0 * Math.PI);
            if (currentAngleRadiansMod < 0.0) {
                currentAngleRadiansMod += 2.0 * Math.PI;
            }

            // The reference angle has the range [0, 2pi) but the Falcon's encoder can go above that
            double adjustedReferenceAngleRadians = referenceAngleRadians + currentAngleRadians - currentAngleRadiansMod;
            if (referenceAngleRadians - currentAngleRadiansMod > Math.PI) {
                adjustedReferenceAngleRadians -= 2.0 * Math.PI;
            } else if (referenceAngleRadians - currentAngleRadiansMod < -Math.PI) {
                adjustedReferenceAngleRadians += 2.0 * Math.PI;
            }

            // does little without next line
//            positionVoltage.withPosition(adjustedReferenceAngleRadians / motorEncoderPositionCoefficient);
//            motor.setControl(positionVoltage);
            // seems to be same as above behavior
//            motor.setPosition(adjustedReferenceAngleRadians / motorEncoderPositionCoefficient);

            this.referenceAngleRadians = referenceAngleRadians;
        }

        @Override
        public double getStateAngle() {
            double motorAngleRadians = motor.getPosition().getValueAsDouble() * motorEncoderPositionCoefficient;
            motorAngleRadians %= 2.0 * Math.PI;
            if (motorAngleRadians < 0.0) {
                motorAngleRadians += 2.0 * Math.PI;
            }

            return motorAngleRadians;
        }

        @Override
        public void setAbsoluteEncoderOffset(){
            absoluteEncoder.setOffset();
        }
    }
}