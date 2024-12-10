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
    //configurtition for the motor begins here
    private TalonFX motorTalon;
    //velcity and voltage control
    private final VelocityVoltage motorTalonVelocityController = new VelocityVoltage(0);
    private final VoltageOut motorTalonVoltageController = new VoltageOut(0);
    private TalonFXConfiguration motorTalonMotorConfiguration = null;
    private Slot0Configs motorRpmGains = new Slot0Configs().withKP(0.36).withKI(0.1).withKD(0.0075).withKV(0.10);

    private Slot0Configs motorTalonMotorHighRpmGains = new Slot0Configs().withKP(0.36).withKI(0.1).withKD(0.0075).withKV(0.10);
    private Slot1Configs motorTalonMotorLowRpmGains = new Slot1Configs().withKP(0.20).withKI(0.2).withKD(0.0075).withKV(0.05);

    private NeutralModeValue outfeedMotorTargetNeutralModeValue = NeutralModeValue.Coast;
    private static final double kMinDeadband = 0.001;
      
    //construct motor and print out the fact that its done.
    public MotorTalon(){
        motorTalon = new TalonFX(Constants.OperatorConstants.motorTalonPort);
        configureMotor();
        System.out.print("Motor is set");
    }
    //WARNING: Will go back and forth unpredictably
    public void motorReverse(){
        //reverse motor
        motorTalon.setControl(this.motorTalonVelocityController.withVelocity(-1));
    }

    public void motorForwards(){
        //forwards motor
        motorTalon.setControl(this.motorTalonVelocityController.withVelocity(1));
    }

    public void motorStop(){
        //stop voltage
        motorTalon.setControl(this.motorTalonVoltageController.withOutput(0));
    }

    //required. I wounldnt reccomend touching but if you know what your doing go ahead.
    private void configureMotor(){
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
        //print out the motor speed for debugging. can be commented out without any damage to the code.
        System.out.println(motorTalon.get());
        super.periodic();
    }
}
