package frc.robot.subsystems;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
//ignore for now
import frc.robot.Constants;


public class TalonMotorSubsystem extends SubsystemBase{
    private final TalonFX motor;
    private final VelocityVoltage velocityController = new VelocityVoltage(0);
    private final VoltageOut voltageController = new VoltageOut(0);
    private Slot0Configs motorRpmGains = new Slot0Configs().withKP(0.36).withKI(0.1).withKD(0.0075).withKV(0.10);
    // Magic numbers copied from Ted ShooterOutfeedMotor
    private TalonFXConfiguration motorConfiguration = null;
    private static final double kMinDeadband = 0.001;
    // Numbers copied from Ted ShooterOutfeedMotor
    private NeutralModeValue outfeedMotorTargetNeutralModeValue = NeutralModeValue.Coast;


    public TalonMotorSubsystem(int canID){
        this.motor = new TalonFX(canID);
        configureMotor();
        velocityController.UpdateFreqHz = 0;
        velocityController.Slot = 0;
    }

    public void setMotorSpeed(double motorSpeed){
        if(Math.abs(motorSpeed) < Constants.joystickDeadband){
            motorSpeed = 0;
        } //This code is meant to implement a deadband
        if(motorSpeed == 0){
            motor.setControl(this.voltageController.withOutput(0));
        }
        else{
            this.motor.setControl(velocityController.withVelocity(motorSpeed));
        }
    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
    }

    private void configureMotor(){
    // Config motor
    motorConfiguration = new TalonFXConfiguration();
    motorConfiguration.MotorOutput.NeutralMode = this.outfeedMotorTargetNeutralModeValue;
    motorConfiguration.MotorOutput.withDutyCycleNeutralDeadband(kMinDeadband);
    motorConfiguration.Slot0 = motorRpmGains;
    // do not config feedbacksource, since the default is the internal one.
    motorConfiguration.Voltage.PeakForwardVoltage = 12;
    motorConfiguration.Voltage.PeakReverseVoltage = -12;
    motorConfiguration.Voltage.SupplyVoltageTimeConstant = Constants.motorSupplyVoltageTimeConstant;
    // maximum current settings
    motorConfiguration.CurrentLimits.StatorCurrentLimit = Constants.motorSupplyCurrentMaximumAmps;
    motorConfiguration.CurrentLimits.StatorCurrentLimitEnable = true;
    motorConfiguration.CurrentLimits.SupplyCurrentLimit = Constants.motorSupplyCurrentMaximumAmps;
    motorConfiguration.CurrentLimits.SupplyCurrentLimitEnable = true;
    // left motor direction
    motorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    StatusCode response = motor.getConfigurator().apply(this.motorConfiguration);
    if (!response.isOK()) {
      System.out.println(
          "TalonFX ID " + motor.getDeviceID() + " failed config with error " + response.toString());
    }

    }

}   

