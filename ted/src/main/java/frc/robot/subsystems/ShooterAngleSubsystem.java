// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: ShooterAngleSubsystem.java
// Intent: Forms the prelminary code for shooter.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StrictFollower;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.control.Constants;
import frc.robot.control.InstalledHardware;
import frc.robot.common.MotorUtils;
import frc.robot.common.ShooterPosition;

/**
 * Forms a class for the shooter subsystem
 * Consists of two outfeed motors, 
 * 1 angle motor, and 1 angle encoder
 */
public class ShooterAngleSubsystem extends SubsystemBase {

  // Shooter gearing 
  private static final double angleMotorGearRatio = 450.0; // 450:1 (100:1 -> 72:16) 
  //private static final double angleMotorGearRatio = 45.0; // remove 1 10x gearbox stage for testing
  private static final double angleEncoderGearRatio = 1.0; // angle encoder is mounted directly onto shaft
  
  private static final double kMinDeadband = 0.001;
  private static final int kPIDLoopIdx = 0;
  private static final double kMaxVoltage = 12;

  private TalonFX angleLeftMotor = new TalonFX(Constants.shooterLeftAngleMotorCanId);
  private TalonFX angleRightMotor = new TalonFX(Constants.shooterRightAngleMotorCanId);
  private CANcoder angleEncoder = new CANcoder(Constants.shooterLeftAngleEncoderCanId);
  private final MotionMagicVoltage angleLeftVoltageController = new MotionMagicVoltage(0);
  // angleRightMotor follows angleLeftMotor, so it doesn't need its own VoltageController
  private boolean shooterIsAtDesiredAngle = true; // don't start moving until angle is set. 
  private double desiredAngleDegrees; 
  private double internalAngleOffsetDegrees = 0; // used when running from intenral motor encoder, ignored when using CanCoder

  // Motor controller gains
  private Slot0Configs angleMotorGains = new Slot0Configs().withKP(80).withKI(0.0).withKD(5.0).withKV(0.12);

  /**
   * Constructor for shooter subsystem
   */
  public ShooterAngleSubsystem() {
    configureAngleEncoder();
    configureAngleMotors();  
    setInternalEncoderOffset();   
    /* Make control requests synchronous */
    angleLeftVoltageController.UpdateFreqHz = 0;
    if (InstalledHardware.shooterRightAngleMotorrInstalled) {
      // set angleRightMotor to strict-follow angleLeftMotor
      // strict followers ignore the leader's invert and use their own
      angleRightMotor.setControl(new StrictFollower(angleLeftMotor.getDeviceID()));
    }
  }

  /**
   * A method to get the shooter angle
   * @return angle in degrees
   */
  public double getAngleDegrees(){
    double offset = InstalledHardware.shooterAngleCanCoderInstalled ? 0 : internalAngleOffsetDegrees;
    return rotationsToDegrees(angleLeftMotor.getPosition().getValue()) + offset;
  }

  /**
   * A method to test whether the angle is within tolerance of the target angle
   * @param targetAngleDegrees
   * @return true if the angle is within tolerance
   */
  public boolean isAngleWithinTolerance(double targetAngleDegrees){
    return Math.abs(getAngleDegrees() - targetAngleDegrees) < Constants.shooterAngleToleranceDegrees;
  }

  /**
   * this method will be called once per scheduler run
   */
  @Override
  public void periodic() {
    if (!shooterIsAtDesiredAngle) {
        // use motionMagic voltage control
        angleLeftMotor.setControl(angleLeftVoltageController.withPosition(degreesToRotations(desiredAngleDegrees)));
        // angleRightMotor acts as a follower
        // keep moving until it reaches target angle
        shooterIsAtDesiredAngle = isAngleWithinTolerance(desiredAngleDegrees);
    }
    SmartDashboard.putNumber("Shooter Absolute Angle Degrees", rotationsToDegrees(angleEncoder.getPosition().getValue()));
    SmartDashboard.putNumber("Shooter Motor Encoder Degrees", getAngleDegrees());
    SmartDashboard.putNumber("Shooter Angle Motor Rotations ", angleLeftMotor.getPosition().getValue());
  }

  /**
   * A method to translate shooter positions into degrees
   * @param position
   * @return degrees
   */
  public double positionToDegrees(ShooterPosition position){
    double angle = Constants.shooterAngleMinDegrees;
    switch (position) {
      case Stow:
        angle = Constants.shooterAngleStowDegrees;
        break;
      case Minimum:
        angle = Constants.shooterAngleMinDegrees;
        break;
      case Medium:
        angle = (Constants.shooterAngleMaxDegrees + Constants.shooterAngleMinDegrees) / 2;
        break;
      case Maximum:
        angle = Constants.shooterAngleMaxDegrees;
        break;
      default:
        angle = Constants.shooterAngleStowDegrees;
        break;
    }
    return angle;
  }

  /**
   * A method to set the shooter angle
   * @param degrees
   */
  public void setAngleDegrees(double degrees){
    // System.out.println("Setting Shooter Angle to " + degrees + " degrees.");
    double clampedDegrees = MotorUtils.clamp(degrees, Constants.shooterAngleMinDegrees, Constants.shooterAngleMaxDegrees);
    if (clampedDegrees != degrees){
      System.out.println("Warning: Shooter Angle requested degrees of " + degrees + 
      "exceeded bounds of [" + Constants.shooterAngleMinDegrees + " .. " + Constants.shooterAngleMaxDegrees +
      "]. Clamped to " + clampedDegrees + ".");
    }
    double offset = InstalledHardware.shooterAngleCanCoderInstalled ? 0 : internalAngleOffsetDegrees;
    desiredAngleDegrees = clampedDegrees - offset;
    shooterIsAtDesiredAngle = false;
  }

  /**
   * A method to set shooter position to desired angle
   * @param position
   */
  public void setPosition(ShooterPosition position) {
    double angle = positionToDegrees(position);
    setAngleDegrees(angle);
  }

  /**
   * This method will be called once per scheduler run during simulation
   */
  @Override
  public void simulationPeriodic() {
  }

  private void configureAngleEncoder() {
    // Config CanCoder
    CANcoderConfiguration encoderConfigs = new CANcoderConfiguration();
    encoderConfigs.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    encoderConfigs.MagnetSensor.MagnetOffset = degreesToRotations(Constants.shooterAbsoluteAngleOffsetDegrees);
    encoderConfigs.MagnetSensor.SensorDirection = Constants.shooterAngleSensorDirection;
    // apply configs
    StatusCode response = angleEncoder.getConfigurator().apply(encoderConfigs);
    if (!response.isOK()) {
      System.out.println(
        "CANcoder ID " + angleEncoder.getDeviceID() + " failed config with error " + response.toString());
    }
  }

  private void configureAngleMotors() {
    // Config angle motor
    TalonFXConfiguration angleConfigs = new TalonFXConfiguration();
    angleConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    angleConfigs.MotorOutput.Inverted = Constants.angleLeftTalonShooterMotorDefaultDirection;
    // FeedbackConfigs and offsets
    angleConfigs.Slot0 = angleMotorGains;
    if (InstalledHardware.shooterAngleCanCoderInstalled) {
      System.out.println("Configuring Shooter Angle Motor with CanCoder Feedback.");
      angleConfigs.Feedback.SensorToMechanismRatio = angleEncoderGearRatio;
      angleConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
      angleConfigs.Feedback.FeedbackRemoteSensorID = Constants.shooterLeftAngleEncoderCanId;
      // offset is set in CanCoder config above
    } 
    else {
      System.out.println("Configuring Shooter Angle Motor with Internal Encoder Feedback.");
      angleConfigs.Feedback.SensorToMechanismRatio = angleMotorGearRatio;
      angleConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor; // the internal encoder
    }
    angleConfigs.MotionMagic.MotionMagicCruiseVelocity = 800.0;
    angleConfigs.MotionMagic.MotionMagicAcceleration = 160;
    angleConfigs.MotionMagic.MotionMagicJerk = 800; 
    // apply configs
    StatusCode response = angleLeftMotor.getConfigurator().apply(angleConfigs);
    if (!response.isOK()) {
      System.out.println(
          "TalonFX ID " + angleLeftMotor.getDeviceID() + " failed config with error " + response.toString());
    }
    // change invert for angleRightMotor
    angleConfigs.MotorOutput.Inverted = Constants.angleRightTalonShooterMotorDefaultDirection;
    // apply configs
    response = angleRightMotor.getConfigurator().apply(angleConfigs);
    if (!response.isOK()) {
      System.out.println(
          "TalonFX ID " + angleRightMotor.getDeviceID() + " failed config with error " + response.toString());
    }
  }

  private void setInternalEncoderOffset(){
    // onlyu call this at startup!
    if (!InstalledHardware.shooterAngleCanCoderInstalled){
      internalAngleOffsetDegrees = Constants.shooterStartingAngleOffsetDegrees - rotationsToDegrees(angleLeftMotor.getPosition().getValue());
    }
  }

  private double degreesToRotations(double degrees)
  {
    return degrees/360;
  }

  private double rotationsToDegrees(double rotations)
  {
    return rotations * 360;
  }

}