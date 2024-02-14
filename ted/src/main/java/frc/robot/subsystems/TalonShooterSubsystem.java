// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: IntakeSubsystem.java
// Intent: Forms the prelminary code for intake subsystem.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.control.Constants;
import frc.robot.common.MotorUtils;
import frc.robot.common.ShooterPosition;

/**
 * Forms a class for the shooter subsystem
 * Consists of two outfeed motors, 
 * 1 angle motor, and 1 angle encoder
 */
public class TalonShooterSubsystem extends SubsystemBase {

  // Talon info
  private static final double velocitySufficientWarmupThreshold = 0.8;

  // Shooter gearing - currently 1:1
  private static final double topShooterGearRatio = 1.0;
  private static final double angleMotorGearRatio = 1.0;
  
  private static final double kMinDeadband = 0.001;
  private static final int kPIDLoopIdx = 0;
  private static final int kTimeoutMs = 30;
  private static final double kMaxVoltage = 12;

  private TalonFX leftTopMotor = new TalonFX(Constants.leftTopTalonShooterMotorCanId);
  final VoltageOut leftVoltageController = new VoltageOut(0);
  private TalonFX rightTopMotor = new TalonFX(Constants.rightTopTalonShooterMotorCanId);
  final VoltageOut rightVoltageController = new VoltageOut(0);
  private TalonFX angleMotor = new TalonFX(Constants.shooterAngleMotorCanId);
  private CANcoder angleEncoder = new CANcoder(Constants.shooterAngleEncoderCanId);
  private final MotionMagicVoltage angleVoltageController = new MotionMagicVoltage(0);

  // Converted old settings to new settings using calculator at:
  // https://v6.docs.ctr-electronics.com/en/stable/docs/migration/migration-guide/closed-loop-guide.html
  // old settings
  // private Gains leftMotorGains = new Gains(0.50, 0.001, 5, 1023/20660.0, 300, 1.00);
  // private Gains rightMotorGains = new Gains(0.50, 0.001, 5, 1023/20660.0, 300, 1.00);
  // new settings
  private Slot0Configs leftMotorGains = new Slot0Configs().withKP(1.2012).withKI(2.4023).withKD(0.0120).withKV(0.1189);
  private Slot0Configs rightMotorGains = new Slot0Configs().withKP(1.2012).withKI(2.4023).withKD(0.0120).withKV(0.1189);
  private Slot0Configs angleMotorGains = new Slot0Configs().withKP(1.2012).withKI(2.4023).withKD(0.0120).withKV(0.1189);

  /**
   * Constructor for shooter subsystem
   */
  public TalonShooterSubsystem() {
    configureOutfeedMotors();
    configureAngleEncoder();
    configureAngleMotor();    
    /* Make control requests synchronous */
    leftVoltageController.UpdateFreqHz = 0;
    rightVoltageController.UpdateFreqHz = 0; 
    angleVoltageController.UpdateFreqHz = 0;

    CommandScheduler.getInstance().registerSubsystem(this);
  }

  /**
   * A method to get the shooter angle
   * @return angle in degrees
   */
  public double getAngleDegrees(){
    return rotationsToDegrees(angleMotor.getPosition().getValue());
  }

  /**
   * A method to get the top left shooter speed
   * @return spped in RPM
   */
  public double getTopLeftSpeedRpm(){
    return rotationsPerSToRpm(leftTopMotor.getVelocity().getValue(), topShooterGearRatio);
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
   * A method to test whether the shooter is at speed
   * @return true if the shooter is at speed
   */
  public boolean isAtSpeed() {
    return (Math.abs(getTopLeftSpeedRpm() - Constants.shooterLeftDefaultSpeedRpm)
        / Constants.shooterLeftDefaultSpeedRpm) < velocitySufficientWarmupThreshold;
  }

  /**
   * this method will be called once per scheduler run
   */
  @Override
  public void periodic() {
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
   * A method to stop shooter outfeed motors
   */
  public void setAllStop() {
    this.setShooterSpeedLeft(0.0);
    this.setShooterSpeedRight(0.0);
  }

  /**
   * A method to set the shooter angle
   * @param degrees
   */
  public void setAngleDegrees(double degrees){
    double clampedDegrees = MotorUtils.clamp(degrees, Constants.shooterAngleMinDegrees, Constants.shooterAngleMaxDegrees);
    if (clampedDegrees != degrees){
      System.out.println("Warning: Shooter Angle requested degrees of " + degrees + 
      "exceeded bounds of [" + Constants.shooterAngleMinDegrees + " .. " + Constants.shooterAngleMaxDegrees +
      "]. Clamped to " + clampedDegrees + ".");
    }
    // use motionMagic voltage control
    angleMotor.setControl(angleVoltageController.withPosition(degreesToRotations(clampedDegrees)));
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
   * A method to set the speed of the shooter left motor
   * @param speed a percentage [0 .. 1]
   */
  public void setShooterSpeedLeft(double speed) {
    leftTopMotor.setControl(leftVoltageController.withOutput(kMaxVoltage * speed));
  }

  /**
   * A method to set the speed of the shooter right motor
   * @param speed a percentage [0 .. 1]
   */
  public void setShooterSpeedRight(double speed) {
    rightTopMotor.setControl(rightVoltageController.withOutput(kMaxVoltage * speed));
  }

  /**
   * Set the top shooter motor to a specific velocity using the in-built PID controller
   * @param revolutionsPerMinute - the RPM that the top motor should spin
   */
  public void setShooterVelocityLeft(double revolutionsPerMinute) {
    System.out.println("attempting left motor percent output");
    final VelocityVoltage velocityController = new VelocityVoltage(0);
    velocityController.Slot = kPIDLoopIdx;

    leftTopMotor.setControl(
        velocityController.withVelocity(this.convertShooterRpmToMotorUnitsPerS(revolutionsPerMinute,
            TalonShooterSubsystem.topShooterGearRatio)));
  }

  /**
   * Set the top shooter motor to a specific velocity using the in-built PID controller
   * @param revolutionsPerMinute - the RPM that the top motor should spin
   */
  public void setShooterVelocityRight(double revolutionsPerMinute) {
    final VelocityVoltage velocityController = new VelocityVoltage(0);
    velocityController.Slot = kPIDLoopIdx;

    rightTopMotor.setControl(
        velocityController.withVelocity(this.convertShooterRpmToMotorUnitsPerS(revolutionsPerMinute,
            TalonShooterSubsystem.topShooterGearRatio)));
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
    encoderConfigs.MagnetSensor.MagnetOffset = degreesToRotations(Constants.shooterAngleOffsetDegrees);
    encoderConfigs.MagnetSensor.SensorDirection = Constants.shooterAngleSensorDirection;
    // apply configs
    StatusCode response = angleEncoder.getConfigurator().apply(encoderConfigs);
    if (!response.isOK()) {
      System.out.println(
        "CANcoder ID " + angleEncoder.getDeviceID() + " failed config with error " + response.toString());
    }
  }

  private void configureAngleMotor() {
    // Config angle motor
    TalonFXConfiguration angleConfigs = new TalonFXConfiguration();
    angleConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    angleConfigs.ClosedLoopGeneral.ContinuousWrap = true;
    angleConfigs.MotorOutput.Inverted = Constants.angleTalonShooterMotorDefaultDirection;
    // FeedbackConfigs
    angleConfigs.Slot0 = angleMotorGains;
    angleConfigs.Feedback.FeedbackRemoteSensorID = Constants.shooterAngleEncoderCanId;
    angleConfigs.Feedback.FeedbackRotorOffset = degreesToRotations(Constants.shooterAngleOffsetDegrees);
    angleConfigs.Feedback.SensorToMechanismRatio = angleMotorGearRatio;
    // TODO not sure how/if this works
    angleConfigs.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
    // copying Motion Magic values from Phoenix Swerve steer motor example
    // https://api.ctr-electronics.com/phoenix6/release/java/src-html/com/ctre/phoenix6/mechanisms/swerve/SwerveModule.html#line.169
    angleConfigs.MotionMagic.MotionMagicCruiseVelocity = 100.0 / angleMotorGearRatio;
    angleConfigs.MotionMagic.MotionMagicAcceleration = angleConfigs.MotionMagic.MotionMagicCruiseVelocity / 0.100;
    angleConfigs.MotionMagic.MotionMagicExpo_kV = 0.12 * angleMotorGearRatio;
    angleConfigs.MotionMagic.MotionMagicExpo_kA = 0.1;
    // apply configs
    StatusCode response = angleMotor.getConfigurator().apply(angleConfigs);
    if (!response.isOK()) {
      System.out.println(
          "TalonFX ID " + angleMotor.getDeviceID() + " failed config with error " + response.toString());
    }
  }

  private void configureOutfeedMotors() {
    // Config left motor
    TalonFXConfiguration talonConfigs = new TalonFXConfiguration();
    talonConfigs.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    talonConfigs.MotorOutput.withDutyCycleNeutralDeadband(kMinDeadband);
    talonConfigs.Slot0 = leftMotorGains;
    // do not config feedbacksource, since the default is the internal one.
    talonConfigs.Voltage.PeakForwardVoltage = 1;
    talonConfigs.Voltage.PeakReverseVoltage = -1;
    // left motor direction
    talonConfigs.MotorOutput.Inverted = Constants.leftTalonShooterMotorDefaultDirection;
    // TODO could not figure out how to set these on new API
    /**
     * leftMotor.configNominalOutputForward(0, TalonShooterSubsystem.kTimeoutMs);
     * leftMotor.configNominalOutputReverse(0, TalonShooterSubsystem.kTimeoutMs);
     */
    // apply configs
    StatusCode response = leftTopMotor.getConfigurator().apply(talonConfigs);
    if (!response.isOK()) {
      System.out.println(
          "TalonFX ID " + leftTopMotor.getDeviceID() + " failed config with error " + response.toString());
    }

    // Config right motor
    // modify left config for right motor
    // right motor goes a different direction
    talonConfigs.MotorOutput.Inverted = Constants.rightTalonShooterMotorDefaultDirection;
    talonConfigs.Slot0 = rightMotorGains;
    // apply configs
    response = rightTopMotor.getConfigurator().apply(talonConfigs);
    if (!response.isOK()) {
      System.out.println(
          "TalonFX ID " + rightTopMotor.getDeviceID() + " failed config with error " + response.toString());
    }
  }

  // V6 lib needs revolutions per second
  private double convertShooterRpmToMotorUnitsPerS(double targetRpm, double targetGearRatio)
  {
    double targetUnitsPerS = 
      MotorUtils.truncateValue(
        targetRpm,
        Constants.talonMaximumRevolutionsPerMinute * -1.0,
        Constants.talonMaximumRevolutionsPerMinute) *
      targetGearRatio / 60.0;
    return targetUnitsPerS;
  }

  private double rotationsPerSToRpm(double rotationsPerS, double targetGearRatio){
    return rotationsPerS / targetGearRatio * 60.0;
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