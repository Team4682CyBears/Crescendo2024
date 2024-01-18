// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: WristSubsystem.java
// Intent: Subsystem to model the wrist.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import frc.robot.common.*;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.RelativeEncoder;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * A class to model the intake wrist - 1 Geared neo550 to move the arm to a
 * up or down position.
 */
public class WristSubsystem extends SubsystemBase {
  // encoder ticks per arm degree (will corelate with gearbox)
  private static final double TICKS_PER_DEGREE = (42 * 48) / 360; 
  // Encoder Tolerence, raise or lower if it bounces or doesn't reach the target.
  private static final double TOLERANCE = 5; 
  private static final double ToleranceDegrees = 1.0;

  private static final double wristMotorSpeedReductionFactor = 0.3;

  private CANSparkMax wristMotor = new CANSparkMax(Constants.wristMotorCanID, MotorType.kBrushless);
  private SparkMaxPIDController wristPidController;
  private RelativeEncoder wristEncoder;
  private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
  private boolean motorInitializedForSmartMotion = false;

  private boolean isWristMotorInverted = false;
  private double requestedWristMotorSpeed = 0.0;
  private WristPosition targetWristPosition = WristPosition.None;

  /**
   * constructor for WristSubsystem subsystem
   */
  public WristSubsystem() {
    CommandScheduler.getInstance().registerSubsystem(this);
  }

  /**
   * A method to set the wrist motor to a certain RPM based on a relative speed
   * input
   * 
   * @param wristSpeed the relative speed -1.0 to 1.0 to run the everyBot arm
   *                   motor at
   */
  public void setWristSpeed(double wristSpeed) {
    this.targetWristPosition = this.getApproximateWristPostionForCurrentAngle();
    this.requestedWristMotorSpeed = MotorUtils.truncateValue(wristSpeed, -1.0, 1.0);
    wristMotor.set(this.requestedWristMotorSpeed);
  }

  /**
   * A method to get the current wrist position
   * @return - position in degrees
   */
  public double getCurrentWristAngle() {
    return wristEncoder.getPosition() / TICKS_PER_DEGREE;
  }

  /**
   * A method to get the target wrist position
   * @return - the target wrist position
   */
  public WristPosition getTargetWristPosition() {
    return this.targetWristPosition;
  }

  /**
   * A method to determine if the movement is complete
   * @return - true if complete
   */
  public boolean isPositionMovementComplete() {
    boolean movementComplete = false;
    WristPosition targetPosition = this.targetWristPosition;
    if (targetPosition == WristPosition.None) {
      movementComplete = true;
    } else {
      double targetAngle = this.getWristAngleForPosition(targetPosition);
      double currentAngle = this.getCurrentWristAngle();
      double deltaAsPos = Math.abs(targetAngle - currentAngle);
      movementComplete = (deltaAsPos <= WristSubsystem.ToleranceDegrees);
    }
    return movementComplete;
  }

  /**
   * A method to set the target wrist position 
   * @param position 
   */
  public void setTargetWristPosition(WristPosition position) {
    this.setWristAngle(this.getWristAngleForPosition(position));
  }

  /**
   * A method called every tick for this subsystem. 
   */
  @Override
  public void periodic() {
    // confirm that the smart motion is setup - no-op after it is setup first time
    this.initializeMotorsSmartMotion();
    this.refreshWristPosition();
  }

  /**
   * A method called every tick in simulation
   */
  @Override
  public void simulationPeriodic() {

  }

  /**
   * A method to get the wrist angle from the position
   * @param position
   * @return - wrist angle
   */
  private double getWristAngleForPosition(WristPosition position) {
    double targetWristAngle = Double.NaN;
    if (position == WristPosition.PickUp) {
      targetWristAngle = Constants.WRIST_ANGLE_PICKUP;
    } else if (position == WristPosition.PositionOne) {
      targetWristAngle = Constants.WRIST_ANGLE_1;
    } else if (position == WristPosition.PositionTwo) {
      targetWristAngle = Constants.WRIST_ANGLE_2;
    } else if (position == WristPosition.PositionThree) {
      targetWristAngle = Constants.WRIST_ANGLE_3;
    }
    return targetWristAngle;
  }

  /**
   * A method to get the wrist position from the current angle. 
   * @return
   */
  private WristPosition getApproximateWristPostionForCurrentAngle() {
    // TODO - this seems very hacky. Is there a way to store the desired position 
    // and just return that, instead?
    double targetWristAngle = this.getCurrentWristAngle();
    WristPosition discoveredPosition = WristPosition.None;

    if (Constants.WRIST_ANGLE_PICKUP + WristSubsystem.TOLERANCE >= targetWristAngle &&
        Constants.WRIST_ANGLE_PICKUP - WristSubsystem.TOLERANCE <= targetWristAngle) {
      discoveredPosition = WristPosition.PickUp;
    } else if (Constants.WRIST_ANGLE_1 + WristSubsystem.TOLERANCE >= targetWristAngle &&
        Constants.WRIST_ANGLE_1 - WristSubsystem.TOLERANCE <= targetWristAngle) {
      discoveredPosition = WristPosition.PositionOne;
    } else if (Constants.WRIST_ANGLE_2 + WristSubsystem.TOLERANCE >= targetWristAngle &&
        Constants.WRIST_ANGLE_2 - WristSubsystem.TOLERANCE <= targetWristAngle) {
      discoveredPosition = WristPosition.PositionTwo;
    } else if (Constants.WRIST_ANGLE_3 + WristSubsystem.TOLERANCE >= targetWristAngle &&
        Constants.WRIST_ANGLE_3 - WristSubsystem.TOLERANCE <= targetWristAngle) {
      discoveredPosition = WristPosition.PositionThree;
    }
    return discoveredPosition;
  }

  /**
   * A method to set the wrist angle
   * @param wristAngle
   */
  private void setWristAngle(double wristAngle) {
    double targetPositionTicks = wristAngle * TICKS_PER_DEGREE;
    double curPosition = wristEncoder.getPosition();

    if (Math.abs(curPosition - targetPositionTicks) > TOLERANCE) {
      // If the current position is outside the tolerance range, use the PID
      // controller to move the wrist to the target position
      wristPidController.setReference(targetPositionTicks, ControlType.kPosition);
    } else {
      // If within tolerance, stop the wrist motor
      wristMotor.set(0);
    }
  }

  /**
   * A method to zero the wrist. 
   */
  public void zeroWrist() {
    wristEncoder.setPosition(0);
  }

  /**
   * A function intended to be called from periodic to update encoder value of the
   * motor.
   */
  private void refreshWristPosition() {
    SmartDashboard.putNumber("EveryArmMotorSpeedRpm", this.wristEncoder.getVelocity());
    SmartDashboard.putNumber("Wrist Angle", this.wristEncoder.getPosition());
  }

  /**
   * a method devoted to establishing proper startup of the jaws motors
   * this method sets all of the key settings that will help in motion magic
   */
  private void initializeMotorsSmartMotion() {
    if (this.motorInitializedForSmartMotion == false) {
      // PID coefficients
      kP = 2e-3;
      kI = 0;
      kD = 0;
      kIz = 0;
      kFF = 0.0000;
      kMaxOutput = wristMotorSpeedReductionFactor;
      kMinOutput = wristMotorSpeedReductionFactor * -1;
      maxRPM = Constants.neoMaximumRevolutionsPerMinute;
      int smartMotionSlot = 0;

      // Smart Motion Coefficients
      maxVel = maxRPM * wristMotorSpeedReductionFactor; // rpm
      maxAcc = maxVel * 2; // 1/2 second to get up to full speed

      wristMotor.restoreFactoryDefaults();
      wristMotor.setIdleMode(IdleMode.kBrake);
      wristMotor.setInverted(this.isWristMotorInverted);
      wristPidController = wristMotor.getPIDController();
      wristEncoder = wristMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor,
          (int) Constants.RevNeoEncoderTicksPerRevolution);
      wristEncoder.setPositionConversionFactor((double) Constants.RevNeoEncoderTicksPerRevolution);

      // set PID coefficients
      wristPidController.setP(kP);
      wristPidController.setI(kI);
      wristPidController.setD(kD);
      wristPidController.setIZone(kIz);
      wristPidController.setFF(kFF);
      wristPidController.setOutputRange(kMinOutput, kMaxOutput);

      wristPidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
      wristPidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
      wristPidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
      wristPidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

      this.motorInitializedForSmartMotion = true;
    }
  }
}
