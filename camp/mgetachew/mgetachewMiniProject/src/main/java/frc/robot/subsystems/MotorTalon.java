package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class MotorTalon extends SubsystemBase {
    private TalonFX motorTalon;
    private double desiredVelocity = 0;
    private double previousVelocity = 0;
    private final VelocityVoltage motorTalonVelocityController = new VelocityVoltage(0);
    private final VoltageOut motorTalonVoltageController = new VoltageOut(0);
    private TalonFXConfiguration motorTalonMotorConfiguration = null;
    private Slot0Configs motorRpmGains = new Slot0Configs().withKP(0.36).withKI(0.1).withKD(0.0075).withKV(0.10);

    private NeutralModeValue outfeedMotorTargetNeutralModeValue = NeutralModeValue.Coast;
    private static final double kMinDeadband = 0.001;

    public MotorTalon() {
        motorTalon = new TalonFX(Constants.OperatorConstants.motorTalonPort);
        configureMotor();
        System.out.print("Motor doing stuff");
    }

    public void motorReverse() {
        System.out.println("Reverse...");
        this.desiredVelocity = -1;
    }

    public void motorForwards() {
        System.out.println("Forward...");
        this.desiredVelocity = 1;
    }

    public void motorStop() {
        System.out.println("Stop.");
        this.desiredVelocity = 0;
    }

    private void configureMotor() {
        // Config motor
        motorTalonMotorConfiguration = new TalonFXConfiguration();
        motorTalonMotorConfiguration.MotorOutput.NeutralMode = this.outfeedMotorTargetNeutralModeValue;
        motorTalonMotorConfiguration.MotorOutput.withDutyCycleNeutralDeadband(kMinDeadband);
        motorTalonMotorConfiguration.Slot0 = motorRpmGains;
        // do not config feedbacksource, since the default is the internal one.
        motorTalonMotorConfiguration.Voltage.PeakForwardVoltage = 12;
        motorTalonMotorConfiguration.Voltage.PeakReverseVoltage = -12;
        motorTalonMotorConfiguration.Voltage.SupplyVoltageTimeConstant = Constants.OperatorConstants.shooterOutfeedSupplyVoltageTimeConstant;
        // maximum current settings
        motorTalonMotorConfiguration.CurrentLimits.StatorCurrentLimit = Constants.OperatorConstants.shooterOutfeedSupplyCurrentMaximumAmps;
        motorTalonMotorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        motorTalonMotorConfiguration.CurrentLimits.SupplyCurrentLimit = Constants.OperatorConstants.shooterOutfeedSupplyCurrentMaximumAmps;
        motorTalonMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        // motor direction
        motorTalonMotorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        StatusCode response = motorTalon.getConfigurator().apply(this.motorTalonMotorConfiguration);
        if (!response.isOK()) {
            System.out.println(
                    "TalonFX ID " + motorTalon.getDeviceID() + " failed config with error " + response.toString());
        }

    }

    @Override
    public void periodic() {
        if (desiredVelocity != previousVelocity) {
            // System.out.println(motorTalon.get());
            if (desiredVelocity == 0) {
                motorTalon.setControl(this.motorTalonVoltageController.withOutput(0));
            } else {
                motorTalon.setControl(this.motorTalonVelocityController.withVelocity(desiredVelocity));
            }
        }
        this.previousVelocity = this.desiredVelocity;
    }
}
