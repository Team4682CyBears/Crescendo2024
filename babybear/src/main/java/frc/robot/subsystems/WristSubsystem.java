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
 * A class intended to model the intake - 1 Geared neo550 to move the arm to a up or down position.
 */
public class WristSubsystem extends SubsystemBase {
   /**********************************************************************
    CONSTANTS
    ************************************************************************/
    private static final double TICKS_PER_DEGREE = 5; //TODO SET THIS encoder ticks per arm degree (will corelate with gearbox)
    private static final double TOLERANCE = 5; // Encoder Tolerence, raise or lower if it bounces or doesn't reach the target.

    // TODO - use something less than 1.0 for testing
    private static final double wristMotorSpeedReductionFactor = 1.0;

    // Wrist gear reduction
    // TODO - get proper values from Simeon/Grayson
    private static final double wristGearReduction = 10.0/1.0;

    /**********************************************************************
    MEMBERS
    ************************************************************************/
    private CANSparkMax wristMotor = new CANSparkMax(Constants.EveryBotPickerMotorCanId, MotorType.kBrushless);
    private SparkMaxPIDController wristPidController;
    private RelativeEncoder wristEncoder;
    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
    private boolean motorInitializedForSmartMotion = false;

    private boolean isWristMotorInverted = false;
    private double requestedWristMotorSpeed = 0.0;

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
     * @param everyBotPickerSpeed the relative speed -1.0 to 1.0 to run the everyBot arm motor at
     */

    public void setPickerRelativeSpeed(double everyBotPickerSpeed) {
      this.requestedWristMotorSpeed = MotorUtils.truncateValue(everyBotPickerSpeed, -1.0, 1.0);
    }

    public void setPickerAngle(double wristAngle) { 
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

    
    
    @Override
    public void periodic() {
      // confirm that the smart motion is setup - no-op after it is setup first time
      this.initializeMotorsSmartMotion();
      this.refreshWristPosition();
      //wristMotor.set(this.requestedWristMotorSpeed * this.wristMotorSpeedReductionFactor);

    }

    @Override
    public void simulationPeriodic() {

    }

    /* *********************************************************************
    PRIVATE METHODS
    ************************************************************************/
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
        kMaxOutput = 1; 
        kMinOutput = -1;
        maxRPM = Constants.neoFiveFiveZeroMaximumRevolutionsPerMinute;
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



