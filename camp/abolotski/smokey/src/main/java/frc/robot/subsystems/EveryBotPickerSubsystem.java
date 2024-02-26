// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: EveryBotPickerSubsystem.java
// Intent: Subsystem to model the every bot picker motor and associated subsystem.
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
 * A class intended to model the picker - checked with Simeon on 02/15/2023 name is now 'picker'
 */
public class EveryBotPickerSubsystem extends SubsystemBase {

    /* *********************************************************************
    CONSTANTS
    ************************************************************************/
    // TODO - use something less than 1.0 for testing
    private static final double neoMotorSpeedReductionFactor = 1.0;

    // EveryBot picker gear reduction
    // TODO - get proper values from Simeon/Grayson
    private static final double everyBotGearReduction = 10.0/1.0;

    /* *********************************************************************
    MEMBERS
    ************************************************************************/
    private CANSparkMax everyBotMotor = new CANSparkMax(Constants.EveryBotPickerMotorCanId, MotorType.kBrushless);
    private SparkMaxPIDController everyBotPidController;
    private RelativeEncoder everyBotEncoder;
    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
    private boolean motorInitalizedForSmartMotion = false;

    private boolean isEveryBotMotorInverted = true;
    private double requestedEveryBotMotorSpeed = 0.0;

    /* *********************************************************************
    CONSTRUCTORS
    ************************************************************************/

    /**
    * constructor for EveryBotPickerSubsystem subsystem
    */
    public EveryBotPickerSubsystem() {
      CommandScheduler.getInstance().registerSubsystem(this);
    }

    /* *********************************************************************
    PUBLIC METHODS
    ************************************************************************/
    /**
     * A method to set the every bot motor to a certain RPM based on a relative speed input
     * @param everyBotPickerSpeed the relative speed -1.0 to 1.0 to run the everyBot arm motor at
     */
    public void setPickerRelativeSpeed(double everyBotPickerSpeed) {
      this.requestedEveryBotMotorSpeed = MotorUtils.truncateValue(everyBotPickerSpeed, -1.0, 1.0);
    }
   
    /**
     * A method to handle periodic processing
     */
    @Override
    public void periodic() {
      // confirm that the smart motion is setup - no-op after it is setup first time
      this.initializeMotorsSmartMotion();
      this.refreshPickerPosition();
      everyBotMotor.set(this.requestedEveryBotMotorSpeed * this.neoMotorSpeedReductionFactor);
    }

    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }

    /* *********************************************************************
    PRIVATE METHODS
    ************************************************************************/
    /**
     * A function intended to be called from perodic to update encoder value of the motor.
     */
    private void refreshPickerPosition() {
      SmartDashboard.putNumber("EveryBotPickerMotorSpeedRpm", this.everyBotEncoder.getVelocity());
      SmartDashboard.putNumber("EveryBotPickerIntakeSpeedRpm", this.everyBotEncoder.getVelocity()/EveryBotPickerSubsystem.everyBotGearReduction);
    }

    // a method devoted to establishing proper startup of the jaws motors
    // this method sets all of the key settings that will help in motion magic
    private void initializeMotorsSmartMotion() {
      if(this.motorInitalizedForSmartMotion == false) { 
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
        maxVel = maxRPM * neoMotorSpeedReductionFactor; // rpm
        maxAcc = maxVel * 2; // 1/2 second to get up to full speed

        everyBotMotor.restoreFactoryDefaults();
        everyBotMotor.setIdleMode(IdleMode.kBrake);
        everyBotMotor.setInverted(this.isEveryBotMotorInverted);
        everyBotPidController = everyBotMotor.getPIDController();
        everyBotEncoder = everyBotMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, (int)Constants.RevNeoEncoderTicksPerRevolution);
        everyBotEncoder.setPositionConversionFactor((double)Constants.RevNeoEncoderTicksPerRevolution);
   
        // set PID coefficients
        everyBotPidController.setP(kP);
        everyBotPidController.setI(kI);
        everyBotPidController.setD(kD);
        everyBotPidController.setIZone(kIz);
        everyBotPidController.setFF(kFF);
        everyBotPidController.setOutputRange(kMinOutput, kMaxOutput);
    
        everyBotPidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        everyBotPidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        everyBotPidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        everyBotPidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

        this.motorInitalizedForSmartMotion = true;
      }
    }

}