// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: Falcon500DriveControllerFactoryBuilder.java
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
import com.ctre.phoenix6.configs.VoltageConfigs;
import com.ctre.phoenix6.controls.VoltageOut;

import frc.robot.control.HardwareConstants;
import frc.robot.swerveLib.ModuleConfiguration;
import frc.robot.swerveLib.ctre.CtreUtils;

public final class Falcon500DriveControllerFactoryBuilder {
    private static final double TICKS_PER_ROTATION = 2048.0;

    private static final int CAN_TIMEOUT_MS = 250;
    private static final int STATUS_FRAME_GENERAL_PERIOD_MS = 250;

    private double nominalVoltage = Double.NaN;
    private double currentLimit = Double.NaN;
    private double statorCurrentLimit = Double.NaN;
    private double supplyVoltageTimeConstant = Double.NaN;

    public Falcon500DriveControllerFactoryBuilder withVoltageCompensation(double nominalVoltage) {
        this.nominalVoltage = nominalVoltage;
        return this;
    }

    public boolean hasVoltageCompensation() {
        return Double.isFinite(nominalVoltage);
    }

    public DriveControllerFactory<ControllerImplementation, Integer> build() {
        return new FactoryImplementation();
    }

    public Falcon500DriveControllerFactoryBuilder withCurrentLimit(double currentLimit) {
        this.currentLimit = currentLimit;
        return this;
    }

    public Falcon500DriveControllerFactoryBuilder withStatorCurrentLimit(double statorCurrentLimit) {
        this.statorCurrentLimit = statorCurrentLimit;
        return this;
    }

    public Falcon500DriveControllerFactoryBuilder withSupplyVoltageTimeConstant(double supplyVoltageTimeConstant) {
        this.supplyVoltageTimeConstant = supplyVoltageTimeConstant;
        return this;
    }

    public boolean hasCurrentLimit() {
        return Double.isFinite(currentLimit);
    }

    public boolean hasStatorCurrentLimit() {
        return Double.isFinite(statorCurrentLimit);
    }

    public boolean hasSupplyVoltageTimeConstant() {
        return Double.isFinite(supplyVoltageTimeConstant);
    }

    private class FactoryImplementation implements DriveControllerFactory<ControllerImplementation, Integer> {
        @Override
        public ControllerImplementation create(Integer driveConfiguration, ModuleConfiguration moduleConfiguration) {

            TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
            CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
            VoltageConfigs voltageConfigs = new VoltageConfigs();
            VoltageOut voltageRequest = new VoltageOut(0);           

            double sensorPositionCoefficient = Math.PI * moduleConfiguration.getWheelDiameter() * moduleConfiguration.getDriveReduction();
            double sensorVelocityCoefficient = sensorPositionCoefficient * 10.0;

            if (hasVoltageCompensation()) {
                voltageRequest.withOutput(nominalVoltage);
            }

            boolean updateMotorConfigForCurrentLimit = false;
            if (hasCurrentLimit()) {
                currentConfigs.withSupplyCurrentLimit(currentLimit);
                currentConfigs.withSupplyCurrentLimitEnable(true);
                updateMotorConfigForCurrentLimit = true;
            }

            if(hasStatorCurrentLimit()) {
                currentConfigs.withStatorCurrentLimit(statorCurrentLimit);
                currentConfigs.withStatorCurrentLimitEnable(true);
                updateMotorConfigForCurrentLimit = true;
            }

            if(updateMotorConfigForCurrentLimit) {
                motorConfiguration.withCurrentLimits(currentConfigs);
            }

            TalonFX motor = new TalonFX(driveConfiguration);

            motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;
            motorConfiguration.MotorOutput.Inverted = (moduleConfiguration.isDriveInverted() ? InvertedValue.CounterClockwise_Positive : InvertedValue.Clockwise_Positive);

            if(hasSupplyVoltageTimeConstant()) {
                voltageConfigs.withSupplyVoltageTimeConstant(supplyVoltageTimeConstant);
                motorConfiguration.withVoltage(voltageConfigs);
            }

            CtreUtils.checkCtreError(motor.getConfigurator().apply(motorConfiguration), "Failed to apply motor configuration!");
            CtreUtils.checkCtreError(motor.setControl(voltageRequest), "Failed to apply motor control!");

            CtreUtils.checkCtreError(
                motor.getPosition().setUpdateFrequency(HardwareConstants.ctreMotorStatusFramePeriodFrequencyHertz, CAN_TIMEOUT_MS),
                "Failed to configure Falcon status frame period"
            );

            return new ControllerImplementation(motor, sensorVelocityCoefficient, sensorPositionCoefficient);
        }
    }

    private class ControllerImplementation implements DriveController {
        private final TalonFX motor;
        private final double sensorVelocityCoefficient;
        private final double sensorPositionCoefficient;
        private final double nominalVoltage = hasVoltageCompensation() ? Falcon500DriveControllerFactoryBuilder.this.nominalVoltage : 12.0;

        private ControllerImplementation(TalonFX motor, double sensorVelocityCoefficient, double sensorPositionCoefficient) {
            this.motor = motor;
            this.sensorVelocityCoefficient = sensorVelocityCoefficient;
            this.sensorPositionCoefficient = sensorPositionCoefficient;
        }

        @Override
        public void setReferenceVoltage(double voltage) {
            motor.setControl(new VoltageOut(voltage));
        }

        @Override
        public double getStateVelocity() {
            return motor.getVelocity().getValueAsDouble() * sensorVelocityCoefficient;
        }

        // TODO - P0 - mike to confirm that position values are correct with basic move 1 meter test
        @Override
        public void setDistance(double value) {
           motor.setPosition(value / sensorPositionCoefficient);
        }

        @Override
        public double getDistance() {
            return motor.getPosition().getValue() * sensorPositionCoefficient;
        }

    }
}