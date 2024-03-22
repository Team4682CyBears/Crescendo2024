// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: ShooterOutfeedSubsystem.java
// Intent: Forms the prelminary code for shooter outfeed.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.control.Constants;
import frc.robot.control.HardwareConstants;
import frc.robot.common.MotorUtils;

/**
 * Forms a class for the shooter subsystem
 * Consists of two outfeed motors, 
 * 1 angle motor, and 1 angle encoder
 */
public class ShooterOutfeedSubsystem extends SubsystemBase {

  // allowable error in velocty as a % of target
  private final double velocityErrorThreshold = 0.05;

  // Shooter gearing - currently 1:1
  private static final double outfeedShooterGearRatio = 1.0;
  
  private static final double kMinDeadband = 0.001;
  private static final int kPIDLoopIdx = 0;

  private double desiredSpeedRpm = 0;

  private TalonFX leftMotor = new TalonFX(Constants.leftTalonShooterMotorCanId);
  private TalonFX rightMotor = new TalonFX(Constants.rightTalonShooterMotorCanId);
  // controllers for setting motors to specific speed with PID control
  private final VelocityVoltage leftVelocityController = new VelocityVoltage(0);
  private final VelocityVoltage rightVelocityController = new VelocityVoltage(0);
  // controllers for setting motors to 0 without PID control
  private final VoltageOut leftVoltageController = new VoltageOut(0);
  private final VoltageOut rightVoltageController = new VoltageOut(0);

  private Slot0Configs leftMotorGains = new Slot0Configs().withKP(0.46).withKI(0.05).withKD(0.0075).withKV(0.11);
  private Slot0Configs rightMotorGains = new Slot0Configs().withKP(0.46).withKI(0.05).withKD(0.0075).withKV(0.11);

  /**
   * Constructor for shooter subsystem
   */
  public ShooterOutfeedSubsystem() {
    configureOutfeedMotors();
    /* Make control requests synchronous */
    leftVelocityController.UpdateFreqHz = 0;
    rightVelocityController.UpdateFreqHz = 0; 
    // set PID slots on velocity controllers
    leftVelocityController.Slot = kPIDLoopIdx;
    rightVelocityController.Slot = kPIDLoopIdx;
  }

  /**
   * A method to get the top left shooter speed
   * @return spped in RPM
   */
  public double getLeftSpeedRpm(){
    return rotationsPerSToRpm(leftMotor.getVelocity().getValue(), outfeedShooterGearRatio);
  }

  /**
   * A method to get the top left shooter speed
   * @return spped in RPM
   */
  public double getRightSpeedRpm(){
    return rotationsPerSToRpm(rightMotor.getVelocity().getValue(), outfeedShooterGearRatio);
  }

  /**
   * A method to test whether the shooter is at speed
   * @return true if the shooter is at speed
   */
  public boolean isAtSpeed() {
    return (Math.abs(getLeftSpeedRpm() - this.desiredSpeedRpm) / this.desiredSpeedRpm) < this.velocityErrorThreshold &&
      (Math.abs(getRightSpeedRpm() - this.desiredSpeedRpm) / this.desiredSpeedRpm) < this.velocityErrorThreshold;
  }

  /**
   * this method will be called once per scheduler run
   */
  @Override
  public void periodic(){
    if (this.desiredSpeedRpm == 0){ // use voltage controller to stop motors
      leftMotor.setControl(this.leftVoltageController.withOutput(0));
      rightMotor.setControl(this.rightVoltageController.withOutput(0));
    }
    else 
    { // use PID controllers to set a speed
      double revsPerS = this.convertShooterRpmToMotorUnitsPerS(desiredSpeedRpm,
      ShooterOutfeedSubsystem.outfeedShooterGearRatio);
      leftMotor.setControl(leftVelocityController.withVelocity(revsPerS));
      rightMotor.setControl(rightVelocityController.withVelocity(revsPerS));
    }
    SmartDashboard.putBoolean("IsShooterRevved?", isAtSpeed());
    SmartDashboard.putNumber("Shooter RPM", getRightSpeedRpm());
  }

  /**
   * A method to stop shooter outfeed motors
   */
  public void setAllStop() {
    this.setShooterVelocity(0.0);
  }

  /**
   * the shooter motors to a specific velocity using the in-built PID controller
   * @param revolutionsPerMinute - the RPM that the shooter motors should spin
   */
  public void setShooterVelocity(double revolutionsPerMinute) {
    this.desiredSpeedRpm = MotorUtils.truncateValue(revolutionsPerMinute, 0, Constants.shooterMaxRpm);
    if (this.desiredSpeedRpm != revolutionsPerMinute) {
      System.out.println("Warning: Shooter requested speed of " + revolutionsPerMinute + 
      "exceeded max speed of" + Constants.shooterMaxRpm +
      ". Clamped to " + this.desiredSpeedRpm + ".");
    }
  }

  /**
   * This method will be called once per scheduler run during simulation
   */
  @Override
  public void simulationPeriodic() {
  }

  private void configureOutfeedMotors() {
    // Config left motor
    TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
    talonConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    talonConfigs.MotorOutput.withDutyCycleNeutralDeadband(kMinDeadband);
    talonConfigs.Slot0 = leftMotorGains;
    // do not config feedbacksource, since the default is the internal one.
    talonConfigs.Voltage.PeakForwardVoltage = 12;
    talonConfigs.Voltage.PeakReverseVoltage = -12;
    talonConfigs.Voltage.SupplyVoltageTimeConstant = HardwareConstants.shooterOutfeedSupplyVoltageTimeConstant;
    // maximum current settings
    talonConfigs.CurrentLimits.StatorCurrentLimit = HardwareConstants.shooterOutfeedStatorCurrentMaximumAmps;
    talonConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    talonConfigs.CurrentLimits.SupplyCurrentLimit = HardwareConstants.shooterOutfeedSupplyCurrentMaximumAmps;
    talonConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    // left motor direction
    talonConfigs.MotorOutput.Inverted = Constants.leftTalonShooterMotorDefaultDirection;
    // apply configs
    StatusCode response = leftMotor.getConfigurator().apply(talonConfigs);
    if (!response.isOK()) {
      System.out.println(
          "TalonFX ID " + leftMotor.getDeviceID() + " failed config with error " + response.toString());
    }

    // Config right motor
    // modify left config for right motor
    // right motor goes a different direction
    talonConfigs.MotorOutput.Inverted = Constants.rightTalonShooterMotorDefaultDirection;
    talonConfigs.Slot0 = rightMotorGains;
    // apply configs
    response = rightMotor.getConfigurator().apply(talonConfigs);
    if (!response.isOK()) {
      System.out.println(
          "TalonFX ID " + rightMotor.getDeviceID() + " failed config with error " + response.toString());
    }
  }

  // V6 lib needs revolutions per second
  private double convertShooterRpmToMotorUnitsPerS(double targetRpm, double targetGearRatio)
  {
    double targetUnitsPerS = 
      MotorUtils.truncateValue(targetRpm, 0, Constants.shooterMaxRpm) *
      targetGearRatio / 60.0;
    return targetUnitsPerS;
  }

  private double rotationsPerSToRpm(double rotationsPerS, double targetGearRatio){
    return rotationsPerS / targetGearRatio * 60.0;
  }

}