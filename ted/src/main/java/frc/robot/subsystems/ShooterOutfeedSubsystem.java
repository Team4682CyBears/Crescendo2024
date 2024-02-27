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
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.control.Constants;
import frc.robot.common.MotorUtils;

/**
 * Forms a class for the shooter subsystem
 * Consists of two outfeed motors, 
 * 1 angle motor, and 1 angle encoder
 */
public class ShooterOutfeedSubsystem extends SubsystemBase {

  // Talon info
  private static final double velocitySufficientWarmupThreshold = 0.8;

  // Shooter gearing - currently 1:1
  private static final double outfeedShooterGearRatio = 1.0;
  
  private static final double kMinDeadband = 0.001;
  private static final int kPIDLoopIdx = 0;
  private static final double kMaxVoltage = 12;

  private TalonFX leftMotor = new TalonFX(Constants.leftTalonShooterMotorCanId);
  final VoltageOut leftVoltageController = new VoltageOut(0);
  private TalonFX rightMotor = new TalonFX(Constants.rightTalonShooterMotorCanId);
  final VoltageOut rightVoltageController = new VoltageOut(0);
  private final MotionMagicVoltage angleLeftVoltageController = new MotionMagicVoltage(0);
  // angleRightMotor follows angleLeftMotor, so it doesn't need its own VoltageController

  // Converted old settings to new settings using calculator at:
  // https://v6.docs.ctr-electronics.com/en/stable/docs/migration/migration-guide/closed-loop-guide.html
  // old settings
  // private Gains leftMotorGains = new Gains(0.50, 0.001, 5, 1023/20660.0, 300, 1.00);
  // private Gains rightMotorGains = new Gains(0.50, 0.001, 5, 1023/20660.0, 300, 1.00);
  // new settings
  private Slot0Configs leftMotorGains = new Slot0Configs().withKP(1.2012).withKI(2.4023).withKD(0.0120).withKV(0.1189);
  private Slot0Configs rightMotorGains = new Slot0Configs().withKP(1.2012).withKI(2.4023).withKD(0.0120).withKV(0.1189);

  /**
   * Constructor for shooter subsystem
   */
  public ShooterOutfeedSubsystem() {
    configureOutfeedMotors();
    /* Make control requests synchronous */
    leftVoltageController.UpdateFreqHz = 0;
    rightVoltageController.UpdateFreqHz = 0; 
  }

  /**
   * A method to get the top left shooter speed
   * @return spped in RPM
   */
  public double getLeftSpeedRpm(){
    return rotationsPerSToRpm(leftMotor.getVelocity().getValue(), outfeedShooterGearRatio);
  }

  /**
   * A method to test whether the shooter is at speed
   * @param shooterLeftTargetSpeedRpm
   * @return true if the shooter is at speed
   */
  public boolean isAtSpeed(double shooterLeftTargetSpeedRpm) {
    return (Math.abs(getLeftSpeedRpm() - shooterLeftTargetSpeedRpm)
        / shooterLeftTargetSpeedRpm) < velocitySufficientWarmupThreshold;
  }

  /**
   * this method will be called once per scheduler run
   */
  @Override
  public void periodic() {}

  /**
   * A method to stop shooter outfeed motors
   */
  public void setAllStop() {
    this.setShooterSpeedLeft(0.0);
    this.setShooterSpeedRight(0.0);
  }

  /**
   * A method to set the speed of the shooter left motor
   * @param speed a percentage [0 .. 1]
   */
  public void setShooterSpeedLeft(double speed) {
    leftMotor.setControl(leftVoltageController.withOutput(kMaxVoltage * speed));
  }

  /**
   * A method to set the speed of the shooter right motor
   * @param speed a percentage [0 .. 1]
   */
  public void setShooterSpeedRight(double speed) {
    rightMotor.setControl(rightVoltageController.withOutput(kMaxVoltage * speed));
  }

  /**
   * the top shooter motor to a specific velocity using the in-built PID controller
   * @param revolutionsPerMinute - the RPM that the top motor should spin
   */
  public void setShooterVelocityLeft(double revolutionsPerMinute) {
    final VelocityVoltage velocityController = new VelocityVoltage(0);
    velocityController.Slot = kPIDLoopIdx;
    double revsPerS = this.convertShooterRpmToMotorUnitsPerS(revolutionsPerMinute,
    ShooterOutfeedSubsystem.outfeedShooterGearRatio);

    // System.out.println("attempting left shooter velocity at " + revsPerS + " revs/s.");
    leftMotor.setControl(velocityController.withVelocity(revsPerS));
  }

  /**
   * Set the top shooter motor to a specific velocity using the in-built PID controller
   * @param revolutionsPerMinute - the RPM that the top motor should spin
   */
  public void setShooterVelocityRight(double revolutionsPerMinute) {
    final VelocityVoltage velocityController = new VelocityVoltage(0);
    velocityController.Slot = kPIDLoopIdx;
    double revsPerS = this.convertShooterRpmToMotorUnitsPerS(revolutionsPerMinute,
    ShooterOutfeedSubsystem.outfeedShooterGearRatio);

    rightMotor.setControl(velocityController.withVelocity(revsPerS));
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
      MotorUtils.truncateValue(
        targetRpm,
        -1.0 * Constants.talonMaximumRevolutionsPerMinute * targetGearRatio / 60.0,
        Constants.talonMaximumRevolutionsPerMinute) *
      targetGearRatio / 60.0;
    return targetUnitsPerS;
  }

  private double rotationsPerSToRpm(double rotationsPerS, double targetGearRatio){
    return rotationsPerS / targetGearRatio * 60.0;
  }

}