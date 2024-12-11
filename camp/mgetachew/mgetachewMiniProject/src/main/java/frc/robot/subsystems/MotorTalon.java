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

public class MotorTalon extends SubsystemBase{
    //configurtition for the motor begins here
    private TalonFX motorTalon;
    //velcity and voltage control
    private final VelocityVoltage motorTalonVelocityController = new VelocityVoltage(0);
    private final VoltageOut motorTalonVoltageController = new VoltageOut(0);
    private TalonFXConfiguration motorTalonMotorConfiguration = null;
    private Slot0Configs motorRpmGains = new Slot0Configs().withKS(0.5);

    private int reverseSpeedRpm = -1;
    private int forwardSpeedRpm = 1;
    private NeutralModeValue motorTargetNeutralModeValue = NeutralModeValue.Coast;
    private static final double kMinDeadband = 0.001;
      
    /**
     * Makes the motor and configures it automatically
     */
    public MotorTalon(){
        motorTalon = new TalonFX(Constants.OperatorConstants.motorTalonPort);
        configureMotor();
        System.out.print("Motor is set");
        motorTalonVelocityController.withSlot(0);
    }
    /**
     * Moves the motor forwards
     */
    public void motorForwards(){
        //forwards motor
        motorTalon.setControl(this.motorTalonVelocityController.withVelocity(forwardSpeedRpm));
    }
    /**
     * Moves the motor in reverse
     */
    public void motorReverse(){
        //reverse motor
        motorTalon.setControl(this.motorTalonVelocityController.withVelocity(reverseSpeedRpm));
    }

    /**
     * Stops the motor.
     */
    public void motorStop(){
        //stop voltage
        motorTalon.setControl(this.motorTalonVoltageController.withOutput(0));
    }

    @Override
    public void periodic() {
        super.periodic();
    }

        /**
     * Quick configuration for a motor
     */
    private void configureMotor(){
        // Config motor
        motorTalonMotorConfiguration = new TalonFXConfiguration();
        motorTalonMotorConfiguration.MotorOutput.NeutralMode = this.motorTargetNeutralModeValue;
        motorTalonMotorConfiguration.MotorOutput.withDutyCycleNeutralDeadband(kMinDeadband);
        motorTalonMotorConfiguration.Slot0 = motorRpmGains;
        // do not config feedbacksource, since the default is the internal one.
        motorTalonMotorConfiguration.Voltage.PeakForwardVoltage = 12;
        motorTalonMotorConfiguration.Voltage.PeakReverseVoltage = -12;
        motorTalonMotorConfiguration.Voltage.SupplyVoltageTimeConstant = Constants.OperatorConstants.motorSupplyVoltageTimeConstant;
        // maximum current settings
        motorTalonMotorConfiguration.CurrentLimits.StatorCurrentLimit = Constants.OperatorConstants.motorSupplyCurrentMaximumAmps;
        motorTalonMotorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
        motorTalonMotorConfiguration.CurrentLimits.SupplyCurrentLimit = Constants.OperatorConstants.motorSupplyCurrentMaximumAmps;
        motorTalonMotorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
        // motor direction
        motorTalonMotorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    
        StatusCode response = motorTalon.getConfigurator().apply(this.motorTalonMotorConfiguration);
        if (!response.isOK()) {
          System.out.println(
              "TalonFX ID " + motorTalon.getDeviceID() + " failed config with error " + response.toString());
        }
    
        }
}
