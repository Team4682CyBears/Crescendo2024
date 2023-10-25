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

import java.lang.invoke.ConstantBootstraps;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/**
 * A class intended to model the intake - 1 Geared neo550 to move the arm to a up or down position.
 */
public class WristSubsystem extends SubsystemBase {
   /**********************************************************************
    CONSTANTS
    ************************************************************************/
    // Wrist gear reduction
    private static final double wristGearReduction = 48/1;
    
    private static final double TICKS_PER_DEGREE = (42 * 48)/360; //TODO SET THIS encoder ticks per arm degree (will corelate with gearbox)
    private static final double TOLERANCE = 5; // Encoder Tolerence, raise or lower if it bounces or doesn't reach the target.
    private static final double ToleranceDegrees = 1.0;

    // TODO - use something less than 1.0 for testing
    private static final double wristMotorSpeedReductionFactor = 1.0;    

    /**********************************************************************
    MEMBERS
    ************************************************************************/
    private CANSparkMax wristMotor = new CANSparkMax(Constants.wristMotorCanID, MotorType.kBrushless);
    private SparkMaxPIDController wristPidController;
    private RelativeEncoder wristEncoder;
    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
    private boolean motorInitializedForSmartMotion = false;

    private boolean isWristMotorInverted = false;
    private double requestedWristMotorSpeed = 0.0;
    private WristPosition targetWristPosition = WristPosition.None;

    /**********************************************************************
    CONSTRUCTORS
    ************************************************************************/

    /**
    * constructor for WristSubsystem subsystem
    */
    public WristSubsystem() {
      CommandScheduler.getInstance().registerSubsystem(this);
    }


    /**********************************************************************
    PUBLIC METHODS
    ************************************************************************/

    /**
     * A method to set the wrist motor to a certain RPM based on a relative speed input
     * @param wristSpeed the relative speed -1.0 to 1.0 to run the everyBot arm motor at
     */
    public void setWristSpeed(double wristSpeed) {
      this.targetWristPosition = this.getApproximateWristPostionForCurrentAngle();
      this.requestedWristMotorSpeed = MotorUtils.truncateValue(wristSpeed, -1.0, 1.0);
      wristMotor.set(this.requestedWristMotorSpeed);
    }

    /**
     * TODO - Need this comment block Owen
     */
    public double getCurrentWristAngle() {
      return wristEncoder.getPosition() / TICKS_PER_DEGREE;
    }

    /**
     * TODO - Need this comment block Owen
     */
    public WristPosition getTargetWristPosition() {
      return this.targetWristPosition;
    }

    /**
     * TODO - Need this comment block Owen
     */
    public boolean isPositionMovementComplete() {
      boolean movementComplete = false;
      WristPosition targetPosition = this.targetWristPosition;
      if(targetPosition == WristPosition.None) {
        movementComplete = true;
      }
      else {
        double targetAngle = this.getWristAngleForPosition(targetPosition);
        double currentAngle = this.getCurrentWristAngle();
        double deltaAsPos = Math.abs(targetAngle-currentAngle);
        movementComplete = (deltaAsPos <= WristSubsystem.ToleranceDegrees);
      }
      return movementComplete;
    }

    /**
     * TODO - Need this comment block Owen
     * @param position
     */
    public void setTargetWristPosition(WristPosition position) {
      this.setWristAngle(this.getWristAngleForPosition(position));
    }
    
    /**
     * TODO - Need this comment block Owen
     */
    @Override
    public void periodic() {
      // confirm that the smart motion is setup - no-op after it is setup first time
      this.initializeMotorsSmartMotion();
      this.refreshWristPosition();
      //wristMotor.set(this.requestedWristMotorSpeed * this.wristMotorSpeedReductionFactor);

    }

    /**
     * TODO - Need this comment block Owen
     */
    @Override
    public void simulationPeriodic() {

    }

    /* *********************************************************************
    PRIVATE METHODS
    ************************************************************************/

    /**
     * TODO - Need this comment block Owen
     */
    private double getWristAngleForPosition(WristPosition position) {
      double targetWristAngle = Double.NaN;
      if(position == WristPosition.PickUp) {
        targetWristAngle = Constants.WRIST_ANGLE_PICKUP;
      }
      else if(position == WristPosition.PositionOne) {
        targetWristAngle = Constants.WRIST_ANGLE_1;
      }
      else if(position == WristPosition.PositionTwo) {
        targetWristAngle = Constants.WRIST_ANGLE_2;
      }
      else if(position == WristPosition.PositionThree) {
        targetWristAngle = Constants.WRIST_ANGLE_3;
      }
      return targetWristAngle;
    }

    /**
     * TODO - Need this comment block Owen
     */
    private WristPosition getApproximateWristPostionForCurrentAngle() {
      double targetWristAngle = this.getCurrentWristAngle();
      WristPosition discoveredPosition = WristPosition.None;

      if(Constants.WRIST_ANGLE_PICKUP + WristSubsystem.TOLERANCE >= targetWristAngle && 
         Constants.WRIST_ANGLE_PICKUP - WristSubsystem.TOLERANCE <= targetWristAngle) {
        discoveredPosition = WristPosition.PickUp;
      }
      else if(Constants.WRIST_ANGLE_1 + WristSubsystem.TOLERANCE >= targetWristAngle && 
              Constants.WRIST_ANGLE_1 - WristSubsystem.TOLERANCE <= targetWristAngle) {
        discoveredPosition = WristPosition.PositionOne;
      }
      else if(Constants.WRIST_ANGLE_2 + WristSubsystem.TOLERANCE >= targetWristAngle && 
              Constants.WRIST_ANGLE_2 - WristSubsystem.TOLERANCE <= targetWristAngle) {
        discoveredPosition = WristPosition.PositionTwo;
      } 
      else if(Constants.WRIST_ANGLE_3 + WristSubsystem.TOLERANCE >= targetWristAngle && 
      Constants.WRIST_ANGLE_3 - WristSubsystem.TOLERANCE <= targetWristAngle) {
    discoveredPosition = WristPosition.PositionThree;
      }

      return discoveredPosition;
    }

    /*
     * TODO - Need this comment block Owen
     */
    private void setWristAngle(double wristAngle) { 
      double targetPositionTicks = wristAngle * TICKS_PER_DEGREE;
      double curPosition = wristEncoder.getPosition();
      
      if (Math.abs(curPosition - targetPositionTicks) > TOLERANCE) {
          // If the current position is outside the tolerance range, use the PID controller to move the wrist to the target position
          wristPidController.setReference(targetPositionTicks, ControlType.kPosition);
      } else {
          // If within tolerance, stop the wrist motor
          wristMotor.set(0);
      }
    }

    /*
     * TODO
     */
    public void zeroWrist(){
      wristEncoder.setPosition(0);
    }

    private void zeroWrist(double position){
      wristEncoder.setPosition(position);
    }

    /**
     * A function intended to be called from periodic to update encoder value of the motor.
     */
    private void refreshWristPosition() {
      SmartDashboard.putNumber("EveryArmMotorSpeedRpm", this.wristEncoder.getVelocity());
      SmartDashboard.putNumber("Wrist Angle", this.wristEncoder.getPosition());
    }

    // a method devoted to establishing proper startup of the jaws motors
    // this method sets all of the key settings that will help in motion magic
    private void initializeMotorsSmartMotion() {
      if(this.motorInitializedForSmartMotion == false) { 
        // PID coefficients
        kP = 2e-4;
        kI = 0;
        kD = 0;
        kIz = 0; 
        kFF = 0.00001; 
        kMaxOutput = 0.75; 
        kMinOutput = -0.75;
        maxRPM = Constants.neoMaximumRevolutionsPerMinute ;
        int smartMotionSlot = 0;
    
        // Smart Motion Coefficients
        maxVel = maxRPM * wristMotorSpeedReductionFactor; // rpm
        maxAcc = maxVel * 2; // 1/2 second to get up to full speed

        wristMotor.restoreFactoryDefaults();
        wristMotor.setIdleMode(IdleMode.kBrake);
        wristMotor.setInverted(this.isWristMotorInverted);
        wristPidController = wristMotor.getPIDController();
        wristEncoder = wristMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, (int)Constants.RevNeoEncoderTicksPerRevolution);
        wristEncoder.setPositionConversionFactor((double)Constants.RevNeoEncoderTicksPerRevolution);
   
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



