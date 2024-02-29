package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkRelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.CorrectableEncoderRevNeoPlusDigitalIoPort;
import frc.robot.common.MotorUtils;
import frc.robot.control.Constants;
import frc.robot.control.InstalledHardware;

public class ClimberSubsystem extends SubsystemBase{

    /* *********************************************************************
    CONSTANTS
    ************************************************************************/
    // expected to be < 1.0 due to encoder granularity being lower for Rev/Neo
    private static final double climberArmsMotorEncoderTicksPerDegree = Constants.RevNeoEncoderTicksPerRevolution / Constants.DegreesPerRevolution; 
    // Based on discussion with XXX on XXX - ~??:1
    private static final double climberArmsMotorToArmEffectiveGearRatio = (50.0/1.0);

    // important - this should be the maximum extension of the arms' hook and it must also be the length of the cord on the spool - in inches!
    private static final double minimumOverageArmHeightInches = -0.1;
    private static final double minimumArmHeightInches = 0.0;
    private static final double maximumArmHeightInches = 19.5;
    private static final double maximumOverageArmHeightInches = 19.6;
    private static final double slowSpeedToleranceInches = 1.5;
    private static final double maximumHeightFromStoredPositionInches = maximumArmHeightInches - minimumArmHeightInches;
    // measurements of spool diameter in 4 discrete ranges
    // intended to be an average measurement of wire/chord on the spool when the spool is 'fractionaly wound'
    // for example when 0-25% of the cord is wound on the spool we need the diameter of the average winding to be placed in climberArmsSpoolDiameterInches0to25
    // TODO remove this logic, since the sting does not overlap itself on Ted spool design. 
    private static final double climberArmsSpoolDiameterInches0to25 = 1.50; 
    private static final double climberArmsSpoolDiameterInches26to50 = 1.51; 
    private static final double climberArmsSpoolDiameterInches51to75 = 1.52; 
    private static final double climberArmsSpoolDiameterInches76to100 = 1.53;
    
    // Based on discussion with XXX
    private static final boolean spoolWindingIsPositiveSparkMaxNeoMotorOutput = true;

    // TODO change this to final speed when everyone is ready for it
    private static final double neoMotorSpeedReductionFactor = 0.3;

    /* *********************************************************************
    MEMBERS
    ************************************************************************/
    // two matched motors - one for each climber side
    private CANSparkMax leftMotor = new CANSparkMax(Constants.leftClimberMotorCanId, MotorType.kBrushless);
    private SparkPIDController leftPidController;
    private RelativeEncoder leftEncoder;
    private CANSparkMax rightMotor = new CANSparkMax(Constants.rightClimberMotorCanId, MotorType.kBrushless);
    private SparkPIDController rightPidController;
    private RelativeEncoder rightEncoder;
    private double kP, kI, kD, kIz, kFF, kMaxOutput, kMinOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr;
    private double motorReferencePosition = 0.0;

    private double motorEncoderTicksAt100 = this.convertClimberArmsHeightToMotorEncoderPosition(maximumHeightFromStoredPositionInches * 1.00);
    private double motorEncoderTicksAt75 = this.convertClimberArmsHeightToMotorEncoderPosition(maximumHeightFromStoredPositionInches * 0.75);
    private double motorEncoderTicksAt50 = this.convertClimberArmsHeightToMotorEncoderPosition(maximumHeightFromStoredPositionInches * 0.50);
    private double motorEncoderTicksAt25 = this.convertClimberArmsHeightToMotorEncoderPosition(maximumHeightFromStoredPositionInches * 0.25);

    private boolean motorsInitalizedForSmartMotion = false;
    private double targetLeftMotorEncoderTicks = 0;
    private double targetRightMotorEncoderTicks = 0;

    private DigitalInput leftMageneticSensor = null;
    private DigitalInput rightMageneticSensor = null;
    private CorrectableEncoderRevNeoPlusDigitalIoPort leftCorrectableCoupling = null;
    private CorrectableEncoderRevNeoPlusDigitalIoPort rightCorrectableCoupling = null;

    /**********************************************************************
    CONSTRUCTORS
    ************************************************************************/
    /**
    * constructor for ClimberArms subsystem
    */
    public ClimberSubsystem() {

        // confirm that the smart motion is setup - no-op after it is setup first time
        this.forceSensorReset();

        if(InstalledHardware.leftClimberSensorInstalled) {
            leftMageneticSensor = new DigitalInput(Constants.leftClimberSensorDioId);
            leftCorrectableCoupling = new CorrectableEncoderRevNeoPlusDigitalIoPort(
                leftEncoder,
                leftMageneticSensor,
                this.convertClimberArmsHeightToMotorEncoderPosition(ClimberSubsystem.minimumArmHeightInches),
                this.convertClimberArmsHeightToMotorEncoderPosition(ClimberSubsystem.minimumArmHeightInches - 0.75), // assume below the reference zero point by 3/4 of an inch
                this.convertClimberArmsHeightToMotorEncoderPosition(ClimberSubsystem.minimumArmHeightInches + 0.75) // assume above the reference zero point by 3/4 of an inch
                );
        }

        if(InstalledHardware.rightClimberSensorInstalled) {
            rightMageneticSensor = new DigitalInput(Constants.rightClimberSensorDioId);
            rightCorrectableCoupling = new CorrectableEncoderRevNeoPlusDigitalIoPort(
                rightEncoder,
                rightMageneticSensor,
                this.convertClimberArmsHeightToMotorEncoderPosition(ClimberSubsystem.minimumArmHeightInches),
                this.convertClimberArmsHeightToMotorEncoderPosition(ClimberSubsystem.minimumArmHeightInches - 0.75), // assume below the reference zero point by 3/4 of an inch
                this.convertClimberArmsHeightToMotorEncoderPosition(ClimberSubsystem.minimumArmHeightInches + 0.75) // assume above the reference zero point by 3/4 of an inch
                );
        }
    }

    /* *********************************************************************
    PUBLIC METHODS
    ************************************************************************/
  
    /**
     * Determine if both climbers are within a certain tolerance of the most recently set target height
     * @param toleranceInches - tolerance in inches from target height
     * @return true if both climbers are within the tolerance specified
     */
    public boolean areClimbersWithinTolerance(double toleranceInches) {
        return (this.isRightClimberWithinTolerance(toleranceInches) && this.isLeftClimberWithinTolerance(toleranceInches));
    }
   
    public void cancelClimberMovement(){
        this.targetLeftMotorEncoderTicks = this.getLeftMotorEncoderPosition();
        this.targetRightMotorEncoderTicks = this.getRightMotorEncoderPosition();
    }
    /**
     * Method to get the maximum height of the climber
     * @return maximumArmHeightInInches - height in inches
     */
    public static double getClimberHeightMaximumInInches() {
        return ClimberSubsystem.maximumArmHeightInches;
    }

    /**
     * Method to get the maximum height of the climber
     * @return minimumArmHeightInInches - height in inches
     */
    public static double getClimberHeightMinimumInInches() {
        return ClimberSubsystem.minimumArmHeightInches;
    }
   
    /**
     * Method to get the right climber height in inches
     * @return current height of climber in inches
     */
    public double getLeftClimberHeightInInches() {
      return this.convertMotorEncoderPositionToClimberArmsHeight(this.getLeftMotorEncoderPosition());
    }

    /**
     * Method to get the right climber height in inches
     * @return current height of climber in inches
     */
    public double getRightClimberHeightInInches() {
      return this.convertMotorEncoderPositionToClimberArmsHeight(this.getRightMotorEncoderPosition());
    }

    /**
     * Determine if the right climber is within a certain tolerance of the most recently set target height
     * @param toleranceInches - tolerance in inches from target height
     * @return true if the climber is within the tolerance specified
     */
    public boolean isLeftClimberWithinTolerance(double toleranceInches) {
        double currentHeight = this.getLeftClimberHeightInInches();
        double currentTargetHeight = this.convertMotorEncoderPositionToClimberArmsHeight(this.targetLeftMotorEncoderTicks);
        System.out.println("Left Climber isDone?  currentHeight: " + currentHeight + " current target height " + currentTargetHeight + "emcoder ticks " + targetLeftMotorEncoderTicks);
        boolean isDone =  (currentHeight >= currentTargetHeight - toleranceInches  && currentHeight <= currentTargetHeight + toleranceInches) ||
            currentHeight >= ClimberSubsystem.maximumOverageArmHeightInches; //TODO PUT THIS BACK!!! || currentHeight <= ClimberSubsystem.minimumOverageArmHeightInches;
        return isDone;
    }

    /**
     * Determine if the right climber is within a certain tolerance of the most recently set target height
     * @param toleranceInches - tolerance in inches from target height
     * @return true if the climber is within the tolerance specified
     */
    public boolean isRightClimberWithinTolerance(double toleranceInches) {
        double currentHeight = this.getRightClimberHeightInInches();
        double currentTargetHeight = this.convertMotorEncoderPositionToClimberArmsHeight(this.targetRightMotorEncoderTicks);
        boolean isDone =  (currentHeight >= currentTargetHeight - toleranceInches  && currentHeight <= currentTargetHeight + toleranceInches) ||
            currentHeight >= ClimberSubsystem.maximumOverageArmHeightInches || currentHeight <= ClimberSubsystem.minimumOverageArmHeightInches;
        return isDone;
    }

    /**
    * This method will be called once per scheduler run
    */
    @Override
    public void periodic() {
        // confirm that the smart motion is setup - no-op after it is setup first time
        this.forceSensorReset();
        // send stuff to shuffleboard
        this.sendStatistics();

        // implementation below attempts to mimmic kitchen drawyer soft close - 
        // essentially as the robot climber nears the top (from below) or bottom (from above) the behavior will be that it slows down 
        double currentLeftHeight = this.getLeftClimberHeightInInches();
        double currentRightHeight = this.getRightClimberHeightInInches();
        double targetLeftHeight = this.convertClimberArmsHeightToMotorEncoderPosition(this.targetLeftMotorEncoderTicks);
        double targetRightHeight = this.convertClimberArmsHeightToMotorEncoderPosition(this.targetRightMotorEncoderTicks);
        boolean isLeftWithinTolerance =  (Math.abs(currentLeftHeight - targetLeftHeight) <= Constants.climberStandardToleranceInches);
        boolean isRightWithinTolerance =  (Math.abs(currentRightHeight - targetRightHeight) <= Constants.climberStandardToleranceInches);

        double leftTargetTicks = targetLeftMotorEncoderTicks;
        double rightTargetTicks = targetRightMotorEncoderTicks;

        // left side
        if(isLeftWithinTolerance) {
            leftTargetTicks = targetLeftMotorEncoderTicks;
        }
        if(targetLeftHeight > currentLeftHeight) {
            if(currentLeftHeight > targetLeftHeight - ClimberSubsystem.slowSpeedToleranceInches) {
                leftTargetTicks = (this.convertClimberArmsHeightToMotorEncoderPosition((currentLeftHeight+targetLeftHeight)/2.0));
            }
        }
        else if(targetLeftHeight < currentLeftHeight) {
            if(currentLeftHeight < targetLeftHeight + ClimberSubsystem.slowSpeedToleranceInches) {
                leftTargetTicks = (this.convertClimberArmsHeightToMotorEncoderPosition((currentLeftHeight+targetLeftHeight)/2.0));
            }
        }
        leftPidController.setReference(leftTargetTicks, ControlType.kSmartMotion);

        // right side
        if(isRightWithinTolerance) {
            rightTargetTicks = targetRightMotorEncoderTicks;
        }
        if(targetRightHeight > currentRightHeight) {
            if(currentRightHeight > targetRightHeight - ClimberSubsystem.slowSpeedToleranceInches) {
                rightTargetTicks = (this.convertClimberArmsHeightToMotorEncoderPosition((currentRightHeight+targetRightHeight)/2.0));
            }
        }
        else if(targetRightHeight < currentRightHeight) {
            if(currentRightHeight < targetRightHeight + ClimberSubsystem.slowSpeedToleranceInches) {
                rightTargetTicks = (this.convertClimberArmsHeightToMotorEncoderPosition((currentRightHeight+targetRightHeight)/2.0));
            }
        }
        rightPidController.setReference(rightTargetTicks, ControlType.kSmartMotion);
    }

    /**
     * Method to update new target position of both climbers
     * @param targetHeightInInches - height in inches
     */
    public void setBothClimberHeightsInInches(double targetHeightInInches) {
        this.setLeftClimberHeightInInches(targetHeightInInches);
        this.setRightClimberHeightInInches(targetHeightInInches);
    }

    /**
     * Method to update new target position of the left climber
     * @param targetHeightInInches - height in inches
     */
    public void setLeftClimberHeightInInches(double targetHeightInInches) {
        targetLeftMotorEncoderTicks = this.convertClimberArmsHeightToMotorEncoderPosition(targetHeightInInches);
    }

    /**
     * Method to update new target position of the right climber
     * @param targetHeightInInches - height in inches
     */
    public void setRightClimberHeightInInches(double targetHeightInInches) {
        targetRightMotorEncoderTicks = this.convertClimberArmsHeightToMotorEncoderPosition(targetHeightInInches);
    }

    /* *********************************************************************
    PRIVATE METHODS
    ************************************************************************/
    // a method to convert a climber arms height into the motor encoder position for the existing setup
    // note this includes adding the originating motor reference position (which is hopefully 0.0)
    private double convertClimberArmsHeightToMotorEncoderPosition(double targetHeightInInches) {
        double climberArmsHeightInInches = MotorUtils.truncateValue(
            targetHeightInInches, 
            0.0,
            ClimberSubsystem.maximumHeightFromStoredPositionInches);

        double remainingFractionUnwound = climberArmsHeightInInches / ClimberSubsystem.maximumHeightFromStoredPositionInches;
        double encoderTargetUnits = this.motorReferencePosition;
        double travelingContributionFraction = 0.0;
        double travelingContributionHeight = 0.0;
        double travelingContributionRevolutions = 0.0;
        double travelingSpoolDiameter = 0.0;

        // build encoder units by looping through 4 different spool diameters
        for (int inx = 0; inx < 4; ++inx)
        {
            // determine the spool diameter based on each quarter of the spool
            switch (inx)
            {
                case (0):
                    travelingSpoolDiameter = ClimberSubsystem.climberArmsSpoolDiameterInches76to100;
                    break;
                case (1):
                    travelingSpoolDiameter = ClimberSubsystem.climberArmsSpoolDiameterInches51to75;
                    break;
                case (2):
                    travelingSpoolDiameter = ClimberSubsystem.climberArmsSpoolDiameterInches26to50;
                    break;
                case (3):
                    travelingSpoolDiameter = ClimberSubsystem.climberArmsSpoolDiameterInches0to25;
                    break;
                default:
                    // should we throw ... using max value will drive revolutions toward 0, so its ok for now
                    travelingSpoolDiameter = Double.MAX_VALUE;
                    break;
            }

            // build up each fractional portion of the spool using its differing diameters due to wire wind
            travelingContributionFraction = remainingFractionUnwound > 0.25 ? 0.25 : remainingFractionUnwound;
            remainingFractionUnwound -= travelingContributionFraction;
            if (travelingContributionFraction > 0.0)
            {
                travelingContributionHeight = travelingContributionFraction * ClimberSubsystem.maximumHeightFromStoredPositionInches;
                travelingContributionRevolutions = travelingContributionHeight / (Math.PI * travelingSpoolDiameter);
                encoderTargetUnits += travelingContributionRevolutions * Constants.DegreesPerRevolution * ClimberSubsystem.climberArmsMotorEncoderTicksPerDegree * ClimberSubsystem.climberArmsMotorToArmEffectiveGearRatio;
            }
        }

        return encoderTargetUnits;
    }

    // a method to convert the current motor encoder position for the existing setup into telescoping arms height 
    // note this includes subtracting the originating motor reference position (which is hopefully 0.0)
    private double convertMotorEncoderPositionToClimberArmsHeight(double climberArmsMotorEncoderPosition) {
        double remainingEncoderTicksUnwound = climberArmsMotorEncoderPosition - this.motorReferencePosition;
        double travelingEncoderMaximumContribution = 0.0;
        double travelingEncoderTicksUnwound = 0.0;
        double travelingSpoolDiameter = 0.0;
        double targetHeightInInches = 0.0;

        // build encoder units by looping through 4 different spool diameters
        for (int inx = 0; inx < 4; ++inx)
        {
            // determine the spool diameter based on each quarter of the spool
            switch (inx)
            {
                case (0):
                    travelingEncoderMaximumContribution = this.motorEncoderTicksAt100 - this.motorEncoderTicksAt75;
                    travelingSpoolDiameter = ClimberSubsystem.climberArmsSpoolDiameterInches76to100;
                    break;
                case (1):
                    travelingEncoderMaximumContribution = this.motorEncoderTicksAt75 - this.motorEncoderTicksAt50;
                    travelingSpoolDiameter = ClimberSubsystem.climberArmsSpoolDiameterInches51to75;
                    break;
                case (2):
                    travelingEncoderMaximumContribution = this.motorEncoderTicksAt50 - this.motorEncoderTicksAt25;
                    travelingSpoolDiameter = ClimberSubsystem.climberArmsSpoolDiameterInches26to50;
                    break;
                case (3):
                    travelingEncoderMaximumContribution = this.motorEncoderTicksAt25 - this.motorReferencePosition;
                    travelingSpoolDiameter = ClimberSubsystem.climberArmsSpoolDiameterInches0to25;
                    break;
                default:
                    // should we throw ... 
                    travelingEncoderMaximumContribution = 0;
                    travelingSpoolDiameter = 0;
                    break;
            }

            // build up each portion of the spool that was unwound
            travelingEncoderTicksUnwound = remainingEncoderTicksUnwound > travelingEncoderMaximumContribution ? travelingEncoderMaximumContribution : remainingEncoderTicksUnwound;
            remainingEncoderTicksUnwound -= travelingEncoderTicksUnwound;
            if (travelingEncoderTicksUnwound > 0.0)
            {
                targetHeightInInches +=
                  (travelingEncoderTicksUnwound / Constants.RevNeoEncoderTicksPerRevolution / ClimberSubsystem.climberArmsMotorToArmEffectiveGearRatio) *
                  (Math.PI * travelingSpoolDiameter);
            }
        }

        return targetHeightInInches;
    }

    /**
     * A method to update the sensors on this device
     */
    private void forceSensorReset() {
        this.initializeMotorsSmartMotion();
        leftEncoder.setPosition(0.0);
        rightEncoder.setPosition(0.0);
    }
 
    private double getLeftMotorEncoderPosition() {
        return leftCorrectableCoupling.getCurrentEncoderPosition();
    }

    private double getLeftMotorOutputSpeed() {
        return leftMotor.getAppliedOutput();
    }

    private String getLeftArmMotionDescription() {
        return this.getArmMotionDescription(this.getLeftMotorOutputSpeed());
    }

    private double getRightMotorEncoderPosition() {
        return rightCorrectableCoupling.getCurrentEncoderPosition();
    }

    private double getRightMotorOutputSpeed() {
        return rightMotor.getAppliedOutput();
    }

    private String getRightArmMotionDescription() {
        return this.getArmMotionDescription(this.getRightMotorOutputSpeed());
    }

    private String getArmMotionDescription(double motorAppliedOutput) {
        double actualMotorOutput = spoolWindingIsPositiveSparkMaxNeoMotorOutput ? motorAppliedOutput : -1.0 * motorAppliedOutput;
        if(actualMotorOutput == 0.0)
        {
            return "Stopped";
        }
        else if(actualMotorOutput > 0.0)
        {
            return "Retracting";
        }
        else if(actualMotorOutput < 0.0)
        {
            return "Extending";
        }
        else
        {
            return "Undefined";
        }
    }

    // a method devoted to establishing proper startup of the jaws motors
    // this method sets all of the key settings that will help in motion magic
    private void initializeMotorsSmartMotion() {
      if(motorsInitalizedForSmartMotion == false)
      {
        // PID coefficients
        kP = 5e-5; 
        kI = 1e-6;
        kD = 0; 
        kIz = 0; 
        kFF = 0.000156; 
        kMaxOutput = 1; 
        kMinOutput = -1;
        maxRPM = Constants.neoMaximumRevolutionsPerMinute;
        int smartMotionSlot = 0;
    
        // Smart Motion Coefficients
        maxVel = maxRPM * neoMotorSpeedReductionFactor; // rpm
        maxAcc = maxVel * 2; // 1/2 second to get up to full speed

        leftMotor.restoreFactoryDefaults();
        leftMotor.setIdleMode(IdleMode.kBrake);
        leftPidController = leftMotor.getPIDController();
        leftEncoder = leftMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, (int)Constants.RevNeoEncoderTicksPerRevolution);
        leftEncoder.setPositionConversionFactor((double)Constants.RevNeoEncoderTicksPerRevolution);
   
        // set PID coefficients
        leftPidController.setP(kP);
        leftPidController.setI(kI);
        leftPidController.setD(kD);
        leftPidController.setIZone(kIz);
        leftPidController.setFF(kFF);
        leftPidController.setOutputRange(kMinOutput, kMaxOutput);
    
        leftPidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        leftPidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        leftPidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        leftPidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

        rightMotor.restoreFactoryDefaults();
        rightMotor.setIdleMode(IdleMode.kBrake);
        rightPidController = rightMotor.getPIDController();
        rightEncoder = rightMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, (int)Constants.RevNeoEncoderTicksPerRevolution);
        rightEncoder.setPositionConversionFactor((double)Constants.RevNeoEncoderTicksPerRevolution);
   
        // set PID coefficients
        rightPidController.setP(kP);
        rightPidController.setI(kI);
        rightPidController.setD(kD);
        rightPidController.setIZone(kIz);
        rightPidController.setFF(kFF);
        rightPidController.setOutputRange(kMinOutput, kMaxOutput);
    
        rightPidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
        rightPidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
        rightPidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
        rightPidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

        this.motorsInitalizedForSmartMotion = true;
      }
    }

    /**
     * Send some statistics to shuffleboard
     */
    public void sendStatistics() {
        SmartDashboard.putNumber("ClimberArmsLeftMotorSpeed", this.getLeftMotorOutputSpeed());
        SmartDashboard.putString("ClimberArmsLeftArmMotionDescription", this.getLeftArmMotionDescription());
        SmartDashboard.putNumber("ClimberArmsLeftClimberHeightInInches", this.getLeftClimberHeightInInches());
        SmartDashboard.putNumber("ClimberArmsLeftEncoderPosition", this.getLeftMotorEncoderPosition());
        SmartDashboard.putNumber("ClimberArmsRightMotorSpeed", this.getRightMotorOutputSpeed());
        SmartDashboard.putString("ClimberArmsRightArmMotionDescription", this.getRightArmMotionDescription());     
        SmartDashboard.putNumber("ClimberArmsRightClimberHeightInInches", this.getRightClimberHeightInInches());
        SmartDashboard.putNumber("ClimberArmsRightEncoderPosition", this.getRightMotorEncoderPosition());
        SmartDashboard.putBoolean("ClimberArmsLeftSensorDetected", this.leftMageneticSensor.get());
        SmartDashboard.putBoolean("ClimberArmsRightSensorDetected", this.rightMageneticSensor.get());
        
    }

}