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
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
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
  private static final int kPIDHighRpmLoopIdx = 0;
  private static final int kPIDLowRpmLoopIdx = 1;

  private double desiredSpeedRpm = 0;

  private TalonFX leftMotor = new TalonFX(Constants.leftTalonShooterMotorCanId);
  private TalonFX rightMotor = new TalonFX(Constants.rightTalonShooterMotorCanId);
  // controllers for setting motors to specific speed with PID control
  private final VelocityVoltage leftVelocityController = new VelocityVoltage(0);
  private final VelocityVoltage rightVelocityController = new VelocityVoltage(0);
  // controllers for setting motors to 0 without PID control
  private final VoltageOut leftVoltageController = new VoltageOut(0);
  private final VoltageOut rightVoltageController = new VoltageOut(0);

  // PID settings for high RPM
  private Slot0Configs leftMotorHighRpmGains = new Slot0Configs().withKP(0.36).withKI(0.1).withKD(0.0075).withKV(0.10);
  private Slot0Configs rightMotorHighRpmGains = new Slot0Configs().withKP(0.36).withKI(0.1).withKD(0.0075).withKV(0.10);
  // PID settings for low RPM
  private Slot1Configs leftMotorLowRpmGains = new Slot1Configs().withKP(0.20).withKI(0.2).withKD(0.0075).withKV(0.05);
  private Slot1Configs rightMotorLowRpmGains = new Slot1Configs().withKP(0.20).withKI(0.2).withKD(0.0075).withKV(0.05);


  // motor configurations
  private TalonFXConfiguration leftMotorConfiguration = null;
  private TalonFXConfiguration rightMotorConfiguration = null;
  // a variable to use as the indicator to enforce brake mode on shooter oufeed motors
  private NeutralModeValue outfeedMotorTargetNeutralModeValue = NeutralModeValue.Coast;

  /**
   * Constructor for shooter subsystem
   */
  public ShooterOutfeedSubsystem() {
    configureOutfeedMotors();
    /* Make control requests synchronous */
    leftVelocityController.UpdateFreqHz = 0;
    rightVelocityController.UpdateFreqHz = 0; 
    // set PID slots on velocity controllers
    leftVelocityController.Slot = kPIDHighRpmLoopIdx;
    rightVelocityController.Slot = kPIDHighRpmLoopIdx;
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
    {
      // using the target speed update brake settings on outfeed motors
      this.enforceBrakeOnTargetSpeed();

      // use PID controllers to set a speed
      double revsPerS = this.convertShooterRpmToMotorUnitsPerS(desiredSpeedRpm,
      ShooterOutfeedSubsystem.outfeedShooterGearRatio);
      leftMotor.setControl(leftVelocityController.withVelocity(revsPerS).withSlot(getPidIdxForTargetSpeed()));
      rightMotor.setControl(rightVelocityController.withVelocity(revsPerS).withSlot(getPidIdxForTargetSpeed()));
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
   * the shooter motors to a specific velocity 
   * uses voltage control if spped is 0, 
   * otherwise, uses the in-built PID controller
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
    this.leftMotorConfiguration = this.getOutfeedMotorConfiguration(
      Constants.leftTalonShooterMotorDefaultDirection,
      leftMotorHighRpmGains, 
      leftMotorLowRpmGains);

    StatusCode response = leftMotor.getConfigurator().apply(this.leftMotorConfiguration);
    if (!response.isOK()) {
      System.out.println(
          "TalonFX ID " + leftMotor.getDeviceID() + " failed config with error " + response.toString());
    }

    // Config right motor
    this.rightMotorConfiguration = this.getOutfeedMotorConfiguration(
      Constants.rightTalonShooterMotorDefaultDirection,
      rightMotorHighRpmGains,
      rightMotorLowRpmGains);
    response = rightMotor.getConfigurator().apply(this.rightMotorConfiguration);
    if (!response.isOK()) {
      System.out.println(
          "TalonFX ID " + rightMotor.getDeviceID() + " failed config with error " + response.toString());
    }
  }

  /**
   * A method to build the motor configuration for shooter outfeed motors
   * @param targetInvert - the desired motor direction
   * @param targetConfigs - the desired motor slot0 configurations
   * @return - the constructed configuration for that target motor
   */
  private TalonFXConfiguration getOutfeedMotorConfiguration(
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
    talonConfigs.Voltage.SupplyVoltageTimeConstant = HardwareConstants.shooterOutfeedSupplyVoltageTimeConstant;
    // maximum current settings
    talonConfigs.CurrentLimits.StatorCurrentLimit = HardwareConstants.shooterOutfeedStatorCurrentMaximumAmps;
    talonConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    talonConfigs.CurrentLimits.SupplyCurrentLimit = HardwareConstants.shooterOutfeedSupplyCurrentMaximumAmps;
    talonConfigs.CurrentLimits.SupplyCurrentLimitEnable = true;
    // left motor direction
    talonConfigs.MotorOutput.Inverted = targetInvert;
    return talonConfigs;
  }

  // V6 lib needs revolutions per second
  private double convertShooterRpmToMotorUnitsPerS(double targetRpm, double targetGearRatio)
  {
    double targetUnitsPerS = 
      MotorUtils.truncateValue(targetRpm, 0, Constants.shooterMaxRpm) *
      targetGearRatio / 60.0;
    return targetUnitsPerS;
  }

  /**
   * A method to return PIDs slot based on target speed on outfeed motors
   * @return the index of the PID to use
   */
  private int getPidIdxForTargetSpeed() {
    return (this.desiredSpeedRpm >= Constants.shooterOutfeedSpeedLowRpmPidThreshold) ? kPIDHighRpmLoopIdx : kPIDLowRpmLoopIdx;
  }
  
  private double rotationsPerSToRpm(double rotationsPerS, double targetGearRatio){
    return rotationsPerS / targetGearRatio * 60.0;
  }

  /**
   * A method to issue brake/non-brake on outfeed motors
   */
  private void enforceBrakeOnTargetSpeed() {

    // only when the configs are already setup should this code run
    if(this.leftMotorConfiguration != null && this.rightMotorConfiguration != null) {

      // when the target speed is less than the threshold then attempt a change
      NeutralModeValue targetNeutralModeValue = 
        ((this.desiredSpeedRpm >= Constants.shooterOutfeedSpeedForcedBrakeThreshold) ? NeutralModeValue.Coast : NeutralModeValue.Brake);

      // only update the brake behavior of motors when state changes 
      if(this.outfeedMotorTargetNeutralModeValue != targetNeutralModeValue) {
        System.out.println("ATTEMPTING UPDATE of mode to " + targetNeutralModeValue.toString());
        this.leftMotorConfiguration.MotorOutput.NeutralMode = targetNeutralModeValue;
        this.rightMotorConfiguration.MotorOutput.NeutralMode = targetNeutralModeValue;
        // want these to send signals down to the motors as close to one another as possible (~atomic)
        StatusCode leftStatus = leftMotor.getConfigurator().apply(this.leftMotorConfiguration);
        StatusCode rightStatus = rightMotor.getConfigurator().apply(this.rightMotorConfiguration);

        // in the event one of the apply updates failed we want to revert it!!
        boolean attemptRevert = (leftStatus != StatusCode.OK || rightStatus != StatusCode.OK);
        if(attemptRevert) {
          System.out.println("REVERTING UPDATE of mode back to " + this.outfeedMotorTargetNeutralModeValue.toString());
          this.leftMotorConfiguration.MotorOutput.NeutralMode = this.outfeedMotorTargetNeutralModeValue;
          this.rightMotorConfiguration.MotorOutput.NeutralMode = this.outfeedMotorTargetNeutralModeValue;

          leftStatus = leftMotor.getConfigurator().apply(this.leftMotorConfiguration);
          rightStatus = rightMotor.getConfigurator().apply(this.rightMotorConfiguration);

          if(leftStatus != StatusCode.OK || rightStatus != StatusCode.OK) {
            System.out.println(
              "MOTOR CONFIG REVERT FAILED , left status == " +
              leftStatus.toString() +
            " right status == " +
            rightStatus.toString());
          }
          else {
            System.out.println("REVERT UPDATE of mode success!");
          }
        }
        // if all goes well update the current state
        else {
          this.outfeedMotorTargetNeutralModeValue = targetNeutralModeValue;
          System.out.println("COMPLETED UPDATE of mode to " + targetNeutralModeValue.toString());
        }
      }
    }
  }

}