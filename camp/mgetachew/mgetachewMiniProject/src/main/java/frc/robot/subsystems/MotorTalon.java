package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.PWMTalonFX;
import edu.wpi.first.wpilibj.motorcontrol.Talon;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix6.hardware.TalonFX;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

public class MotorTalon extends SubsystemBase{
    private TalonFX motorTalon = new TalonFX(Constants.OperatorConstants.motorTalonPort);
    private final VelocityVoltage motorTalonVelocityController = new VelocityVoltage(0);
    private final VoltageOut motorTalonVoltageController = new VoltageOut(0);
    private TalonFXConfiguration motorTalonMotorConfiguration = null;

    private Slot0Configs motorTalonMotorHighRpmGains = new Slot0Configs().withKP(0.36).withKI(0.1).withKD(0.0075).withKV(0.10);
    private Slot1Configs motorTalonMotorLowRpmGains = new Slot1Configs().withKP(0.20).withKI(0.2).withKD(0.0075).withKV(0.05);

    private NeutralModeValue outfeedMotorTargetNeutralModeValue = NeutralModeValue.Coast;
    private static final double kMinDeadband = 0.001;
      

    public MotorTalon(){
        System.out.print("Motor doing stuff");
    }

    public void motorReverse(){
        motorTalon.setControl(this.motorTalonVelocityController.withVelocity(-1));
    }

    public void motorForwards(){
        motorTalon.setControl(this.motorTalonVelocityController.withVelocity(1));
    }

    public void motorStop(){
        motorTalon.setControl(this.motorTalonVoltageController.withOutput(0));
    }

    private void configurefeedMotors() {
        StatusCode response = motorTalon.getConfigurator().apply(this.motorTalonMotorConfiguration);
        this.motorTalonMotorConfiguration = this.getMotorConfiguration(
          Constants.OperatorConstants.rightTalonShooterMotorDefaultDirection,
          motorTalonMotorHighRpmGains,
          motorTalonMotorLowRpmGains);
        response = motorTalon.getConfigurator().apply(this.motorTalonMotorConfiguration);
        if (!response.isOK()) {
          System.out.println(
              "TalonFX ID " + motorTalon.getDeviceID() + " failed config with error " + response.toString());
        }
    }

    private TalonFXConfiguration getMotorConfiguration(
        InvertedValue targetInvert,
        Slot0Configs targetSlot0Configs,
        Slot1Configs targetSlot1Configs) {
    
        // Config left motor
        TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
        talonConfigs.MotorOutput.NeutralMode = this.outfeedMotorTargetNeutralModeValue;
        talonConfigs.MotorOutput.withDutyCycleNeutralDeadband(kMinDeadband);
        talonConfigs.Slot0 = targetSlot0Configs;
        talonConfigs.Slot1 = targetSlot1Configs;
        // do not config feedbacksource, since the default is the internal one.
        talonConfigs.Voltage.PeakForwardVoltage = 12;
        talonConfigs.Voltage.PeakReverseVoltage = -12;
        talonConfigs.Voltage.SupplyVoltageTimeConstant = Constants.OperatorConstants.shooterOutfeedSupplyVoltageTimeConstant;
        // maximum current settings
        talonConfigs.CurrentLimits.StatorCurrentLimit = Constants.OperatorConstants.shooterOutfeedStatorCurrentMaximumAmps;
        talonConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
        talonConfigs.CurrentLimits.SupplyCurrentLimit = Constants.OperatorConstants.shooterOutfeedSupplyCurrentMaximumAmps;
        talonConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
        // left motor direction
        talonConfigs.MotorOutput.Inverted = targetInvert;
        return talonConfigs;
      }
    

    @Override
    public void periodic() {
        System.out.println(motorTalon.get());
        super.periodic();
    }
}
