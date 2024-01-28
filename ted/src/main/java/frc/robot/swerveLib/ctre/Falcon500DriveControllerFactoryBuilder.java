
// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: Falcon500DriveControllerFactoryBuilder.java
// Intent: Same name file port of Swerve Drive Specalties codebase from phoenix5 
// to phoenix6
// SDS codebase found at: 
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.swerveLib.ctre;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;

import frc.robot.swerveLib.DriveController;
import frc.robot.swerveLib.DriveControllerFactory;
import frc.robot.swerveLib.ModuleConfiguration;

public final class Falcon500DriveControllerFactoryBuilder {
    private static final double TICKS_PER_ROTATION = 2048.0;

    private static final int CAN_TIMEOUT_MS = 250;
    private static final int STATUS_FRAME_GENERAL_PERIOD_MS = 250;

    private double nominalVoltage = Double.NaN;
    private double currentLimit = Double.NaN;

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

    public boolean hasCurrentLimit() {
        return Double.isFinite(currentLimit);
    }

    private class FactoryImplementation implements DriveControllerFactory<ControllerImplementation, Integer> {
        @Override
        public ControllerImplementation create(Integer driveConfiguration, ModuleConfiguration moduleConfiguration) {

            // NEW: next 3 lines ...
            TalonFXConfiguration motorConfiguration = new TalonFXConfiguration();
            CurrentLimitsConfigs currentConfigs = new CurrentLimitsConfigs();
            VoltageOut voltageRequest = new VoltageOut(0);           

            double sensorPositionCoefficient = Math.PI * moduleConfiguration.getWheelDiameter() * moduleConfiguration.getDriveReduction() / TICKS_PER_ROTATION;
            double sensorVelocityCoefficient = sensorPositionCoefficient * 10.0;

            if (hasVoltageCompensation()) {
// WAS:                motorConfiguration.voltageCompSaturation = nominalVoltage;
                voltageRequest.withOutput(nominalVoltage);
            }

            if (hasCurrentLimit()) {
// WAS:                motorConfiguration.supplyCurrLimit.currentLimit = currentLimit;
// WAS:                motorConfiguration.supplyCurrLimit.enable = true;
                currentConfigs.withSupplyCurrentLimit(currentLimit);
                currentConfigs.withSupplyCurrentLimitEnable(true);
                motorConfiguration.withCurrentLimits(currentConfigs);
            }

            TalonFX motor = new TalonFX(driveConfiguration);
// WAS:            CtreUtils.checkCtreError(motor.configAllSettings(motorConfiguration), "Failed to configure Falcon 500");
            CtreUtils.checkCtreError(motor.getConfigurator().apply(motorConfiguration), "Failed to apply motor configuration!");
            CtreUtils.checkCtreError(motor.setControl(voltageRequest), "Failed to apply motor configuration!");

            /*
            // according to https://pro.docs.ctr-electronics.com/en/latest/docs/migration/migration-guide/control-requests-guide.html#using-control-requests it seems this call is no longer necessary
WAS:
            if (hasVoltageCompensation()) {
                // Enable voltage compensation
                motor.enableVoltageCompensation(true);
            }
            */

// WAS:            motor.setNeutralMode(NeutralMode.Brake);
            motorConfiguration.MotorOutput.NeutralMode = NeutralModeValue.Brake;

// WAS:            motor.setInverted(moduleConfiguration.isDriveInverted() ? TalonFXInvertType.Clockwise : TalonFXInvertType.CounterClockwise);
            motorConfiguration.MotorOutput.Inverted = (moduleConfiguration.isDriveInverted() ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive);

            // seems like according to https://v6.docs.ctr-electronics.com/en/2023-v6/docs/migration/migration-guide/feature-replacements-guide.html that this call is no longer needed in phenoix 6
// WAS:            motor.setSensorPhase(true);

            // Reduce CAN status frame rates
/* WAS:
            CtreUtils.checkCtreError(
                    motor.setStatusFramePeriod(
                            StatusFrameEnhanced.Status_1_General,
                            STATUS_FRAME_GENERAL_PERIOD_MS,
                            CAN_TIMEOUT_MS
                    ),
                    "Failed to configure Falcon status frame period"
            );
*/
            // See: https://v6.docs.ctr-electronics.com/en/2023-v6/docs/migration/migration-guide/status-signals-guide.html#changing-update-frequency-status-frame-period for frame period update
            CtreUtils.checkCtreError(
                motor.getPosition().setUpdateFrequency(STATUS_FRAME_GENERAL_PERIOD_MS, CAN_TIMEOUT_MS),
                "Failed to configure Falcon status frame period"
            );

            return new ControllerImplementation(motor, sensorVelocityCoefficient);
        }
    }

    private class ControllerImplementation implements DriveController {
        private final TalonFX motor;
        private final double sensorVelocityCoefficient;
        private final double nominalVoltage = hasVoltageCompensation() ? Falcon500DriveControllerFactoryBuilder.this.nominalVoltage : 12.0;

        private ControllerImplementation(TalonFX motor, double sensorVelocityCoefficient) {
            this.motor = motor;
            this.sensorVelocityCoefficient = sensorVelocityCoefficient;
        }

        @Override
        public void setReferenceVoltage(double voltage) {
// WAS:            motor.set(TalonFXControlMode.PercentOutput, voltage / nominalVoltage);
            motor.setControl(new VoltageOut(voltage));
        }

        @Override
        public double getStateVelocity() {
// WAS:            return motor.getSelectedSensorVelocity() * sensorVelocityCoefficient;
            return motor.getVelocity().getValueAsDouble() * sensorVelocityCoefficient;
        }

    }
}
