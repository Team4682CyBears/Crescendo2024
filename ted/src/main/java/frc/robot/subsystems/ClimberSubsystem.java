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
    private static final double climberArmsMotorToArmEffectiveGearRatio = (60.0/1.0);

    private static final double minimumArmHeightInches = 0.0;
    private static final double maximumArmHeightInches = Constants.climberArmToPositionFullDeploy;
    private static final double maximumHeightFromStoredPositionInches = maximumArmHeightInches - minimumArmHeightInches;
    // measurements of spool diameter in 4 discrete ranges
    // intended to be an average measurement of wire/chord on the spool when the spool is 'fractionaly wound'
    // for example when 0-25% of the cord is wound on the spool we need the diameter of the average winding to be placed in climberArmsSpoolDiameterInches0to25
    // TODO - remove this logic at some point, since the sting does not overlap itself on Ted spool design. 
    private static final double climberArmsSpoolDiameterInches0to25 = 1.216; 
    private static final double climberArmsSpoolDiameterInches26to50 = 1.216; 
    private static final double climberArmsSpoolDiameterInches51to75 = 1.216; 
    private static final double climberArmsSpoolDiameterInches76to100 = 1.216;
    
    // Based on testing
    private static final boolean spoolWindingIsPositiveSparkMaxNeoMotorOutput = true;
    private static final double neoMotorSpeedReductionFactor = 1.0;

    private static final double lengthClimberExtensionVeryCloseToEndInches = maximumArmHeightInches - 1.5;
    private static final double lengthClimberExtensionVeryCloseToStopInches = 1.5;
    private static final double neoMotorSpeedReductionFactorVeryCloseToStop = 0.75; 

    private static final double leftClimberSensorResetExtendSpeed = 0.9;
    private static final double leftClimberSensorResetRetractSpeed = -1.0 * leftClimberSensorResetExtendSpeed;
    private static final double rightClimberSensorResetExtendSpeed = leftClimberSensorResetExtendSpeed;
    private static final double rightClimberSensorResetRetractSpeed = leftClimberSensorResetRetractSpeed;

    /* *********************************************************************
    MEMBERS
    ************************************************************************/
    // two matched motors - one for each climber side
    private CANSparkMax leftMotor;
    private SparkPIDController leftPidController;
    private RelativeEncoder leftEncoder;
    private double kPLeft, kILeft, kDLeft, kIzLeft, kFFLeft, kMaxOutputLeft, kMinOutputLeft, maxRPMLeft, maxVelLeft, minVelLeft, maxAccLeft, allowedErrLeft;
    private boolean isLeftMotorInverted = false;
    private DigitalInput leftMageneticSensor = null;
    private CorrectableEncoderRevNeoPlusDigitalIoPort leftCorrectableCoupling = null;
    private boolean leftClimberReady = InstalledHardware.leftClimberInstalled && InstalledHardware.leftClimberSensorInstalled;
    private double targetLeftClimberInches = 0;
    private double targetLeftMotorSpeed = 0;

    private CANSparkMax rightMotor;
    private SparkPIDController rightPidController;
    private RelativeEncoder rightEncoder;
    private double kPRight, kIRight, kDRight, kIzRight, kFFRight, kMaxOutputRight, kMinOutputRight, maxRPMRight, maxVelRight, minVelRight, maxAccRight, allowedErrRight;
    private boolean isRightMotorInverted = true;
    private DigitalInput rightMageneticSensor = null;
    private CorrectableEncoderRevNeoPlusDigitalIoPort rightCorrectableCoupling = null;
    private boolean rightClimberReady = InstalledHardware.rightClimberInstalled && InstalledHardware.rightClimberSensorInstalled;
    private double targetRightClimberInches = 0;
    private double targetRightMotorSpeed = 0;

    private double motorReferencePosition = 0.0;
    private boolean motorsInitalizedForSmartMotion = false;
    private boolean movementWithinTolerance = false;

    private double motorEncoderTicksAt100 = this.convertClimberArmsHeightToMotorEncoderPosition(maximumHeightFromStoredPositionInches * 1.00);
    private double motorEncoderTicksAt75 = this.convertClimberArmsHeightToMotorEncoderPosition(maximumHeightFromStoredPositionInches * 0.75);
    private double motorEncoderTicksAt50 = this.convertClimberArmsHeightToMotorEncoderPosition(maximumHeightFromStoredPositionInches * 0.50);
    private double motorEncoderTicksAt25 = this.convertClimberArmsHeightToMotorEncoderPosition(maximumHeightFromStoredPositionInches * 0.25);

    private long counter = 0;
    private boolean inSpeedMode = true;

    /**********************************************************************
    CONSTRUCTORS
    ************************************************************************/
    /**
    * constructor for ClimberArms subsystem
    */
    public ClimberSubsystem() {

        if(this.leftClimberReady) {
            leftMotor = new CANSparkMax(Constants.leftClimberMotorCanId, MotorType.kBrushless);
        }

        if(this.rightClimberReady) {
            rightMotor = new CANSparkMax(Constants.rightClimberMotorCanId, MotorType.kBrushless);
        }

        // confirm that the smart motion is setup - no-op after it is setup first time
        // and also do the sensor reset
        this.forceSensorReset();

        if(this.leftClimberReady) {
            leftMageneticSensor = new DigitalInput(Constants.leftClimberSensorDioId);
            leftCorrectableCoupling = new CorrectableEncoderRevNeoPlusDigitalIoPort(
                leftEncoder,
                leftMageneticSensor,
                this.convertClimberArmsHeightToMotorEncoderPosition(ClimberSubsystem.minimumArmHeightInches + Constants.climberArmSensorPosition),
                this.convertClimberArmsHeightToMotorEncoderPosition(ClimberSubsystem.minimumArmHeightInches - Constants.climberArmSensorBlindFindDistance),
                this.convertClimberArmsHeightToMotorEncoderPosition(ClimberSubsystem.minimumArmHeightInches + Constants.climberArmSensorBlindFindDistance)
                );
        }

        if(this.rightClimberReady) {
            rightMageneticSensor = new DigitalInput(Constants.rightClimberSensorDioId);
            rightCorrectableCoupling = new CorrectableEncoderRevNeoPlusDigitalIoPort(
                rightEncoder,
                rightMageneticSensor,
                this.convertClimberArmsHeightToMotorEncoderPosition(ClimberSubsystem.minimumArmHeightInches + Constants.climberArmSensorPosition),
                this.convertClimberArmsHeightToMotorEncoderPosition(ClimberSubsystem.minimumArmHeightInches - Constants.climberArmSensorBlindFindDistance),
                this.convertClimberArmsHeightToMotorEncoderPosition(ClimberSubsystem.minimumArmHeightInches + Constants.climberArmSensorBlindFindDistance)
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
   
    /**
     * Method to get the maximum height of the climber
     * @return maximumArmHeightInches - height in inches
     */
    public static double getClimberHeightMaximumInInches() {
        return ClimberSubsystem.maximumArmHeightInches;
    }

    /**
     * Method to get the minimum height of the climber
     * @return minimumArmHeightInches - height in inches
     */
    public static double getClimberHeightMinimumInInches() {
        return ClimberSubsystem.minimumArmHeightInches;
    }
   
    /**
     * Method to get the right climber height in inches
     * @return current height of climber in inches
     */
    public double getLeftClimberHeightInInches() {
        if(this.leftClimberReady){
            return this.convertMotorEncoderPositionToClimberArmsHeight(this.getLeftMotorEncoderPosition());
        }
        return Double.NaN;
    }

    /**
     * Method to get the right climber height in inches
     * @return current height of climber in inches
     */
    public double getRightClimberHeightInInches() {
        if(this.rightClimberReady){
            return this.convertMotorEncoderPositionToClimberArmsHeight(this.getRightMotorEncoderPosition());
        }
        return Double.NaN;
    }

    /**
     * Method to confirm both climbers have found their sensor reset positions
     * @return true if both climbers have moved past their sensor reset positions
     */
    public boolean haveClimbersFoundSensorReset() {
        return (this.leftClimberReady ? this.leftCorrectableCoupling.getMotorEncoderEverReset() : true) &&
               (this.rightClimberReady ? this.rightCorrectableCoupling.getMotorEncoderEverReset() : true);
    }
    
    /**
     * Determine if the right climber is within a certain tolerance of the most recently set target height
     * @param toleranceInches - tolerance in inches from target height
     * @return true if the climber is within the tolerance specified
     */
    public boolean isLeftClimberWithinTolerance(double toleranceInches) {
        return Math.abs(this.getLeftClimberHeightInInches() - this.targetLeftClimberInches) < toleranceInches;
    }

    /**
     * Determine if the right climber is within a certain tolerance of the most recently set target height
     * @param toleranceInches - tolerance in inches from target height
     * @return true if the climber is within the tolerance specified
     */
    public boolean isRightClimberWithinTolerance(double toleranceInches) {
        return Math.abs(this.getRightClimberHeightInInches() - this.targetRightClimberInches) < toleranceInches;
    }

    /**
     * Method to move both arms toward the sensor reset positions of each arm
     * will automatically stop the arm movements when sensor reset has occurred
     */
    public void moveClimbersToSensorReset() {
        // get ready for the arm drive speeds
        double leftSpeed = 0.0;
        double rightSpeed = 0.0;
  
        // left arm
        if(this.leftClimberReady && // the sensor must be present to ever get any left speed other than zero
           this.leftCorrectableCoupling.getMotorEncoderEverReset() == false) { // when the motor encoder has been reset, no need to move the arm
            // extend if the sensor is currently triggered
            // retract if the sensor is NOT currently triggered
            leftSpeed = 
              (this.leftMageneticSensor.get() == false) ? // sensor light is illuminated when .get returns false
              ClimberSubsystem.leftClimberSensorResetExtendSpeed :
              ClimberSubsystem.leftClimberSensorResetRetractSpeed;
        }
  
        // right arm
        if(this.rightClimberReady && // sensor must be present to get any speed
           this.rightCorrectableCoupling.getMotorEncoderEverReset() == false) { // when the motor encoder has been reset, no need to move the arm
            // extend if the sensor is currently triggered
            // retract if the sensor is NOT currently triggered
            rightSpeed =
              (this.rightMageneticSensor.get() == false) ? // sensor light is illuminated when .get returns false
              ClimberSubsystem.rightClimberSensorResetExtendSpeed :
              ClimberSubsystem.rightClimberSensorResetRetractSpeed;
        }
        this.setClimberSpeeds(leftSpeed, rightSpeed);
    }

    /**
    * This method will be called once per scheduler run
    */
    @Override
    public void periodic() {
        // incremenet the counter
        ++counter;

        // confirm that the smart motion is setup - no-op after it is setup first time
        this.initializeMotorsSmartMotion();

        // determine if the movement is in the stop range
        // stop range implies any of the following:
        // magnetic sensor triggered 
        // arm deployed >= limit (e.g., maximumXXXArmExtensionMeters)
        double currentLeftExtensionInInches = this.getLeftClimberHeightInInches();
        boolean isLeftArmAtOrBelowLowStop = (currentLeftExtensionInInches <= 0.0);
        boolean isLeftArmAtOrAboveHighStop = currentLeftExtensionInInches >= maximumArmHeightInches;
        double currentRightExtensionInInches = this.getRightClimberHeightInInches();
        boolean isRightArmAtOrBelowLowStop = (currentRightExtensionInInches <= 0.0);
        boolean isRightArmAtOrAboveHighStop = currentRightExtensionInInches >= maximumArmHeightInches;

        // if we are in speed mode always set motor speeds using motor set
        if(this.inSpeedMode) {
            // Left
            if(this.leftClimberReady) {
                if(isLeftArmAtOrBelowLowStop && this.targetLeftMotorSpeed < 0.0 && this.leftCorrectableCoupling.getMotorEncoderEverReset()) {
                    this.leftMotor.set(0.0);
                }
                else if(isLeftArmAtOrAboveHighStop && this.targetLeftMotorSpeed > 0.0) {
                    this.leftMotor.set(0.0);
                }
                else if(
                    (currentLeftExtensionInInches < ClimberSubsystem.lengthClimberExtensionVeryCloseToStopInches && this.targetLeftMotorSpeed < 0.0 ) ||
                    (currentLeftExtensionInInches > ClimberSubsystem.lengthClimberExtensionVeryCloseToEndInches && this.targetLeftMotorSpeed > 0.0 )) {
                    this.leftMotor.set(this.targetLeftMotorSpeed * neoMotorSpeedReductionFactorVeryCloseToStop);
                }
                else {
                    this.leftMotor.set(this.targetLeftMotorSpeed * neoMotorSpeedReductionFactor);
                }
            }
            // Right
            if(this.rightClimberReady) {
                if(isRightArmAtOrBelowLowStop && this.targetRightMotorSpeed < 0.0 && this.rightCorrectableCoupling.getMotorEncoderEverReset()) {
                    this.rightMotor.set(0.0);
                }
                else if(isRightArmAtOrAboveHighStop && this.targetRightMotorSpeed > 0.0) {
                    this.rightMotor.set(0.0);
                }
                else if(
                    (currentRightExtensionInInches < ClimberSubsystem.lengthClimberExtensionVeryCloseToStopInches && this.targetRightMotorSpeed < 0.0 ) ||
                    (currentRightExtensionInInches > ClimberSubsystem.lengthClimberExtensionVeryCloseToEndInches && this.targetRightMotorSpeed > 0.0 )) {
                    this.rightMotor.set(this.targetRightMotorSpeed * neoMotorSpeedReductionFactorVeryCloseToStop);
                }
                else {
                    this.rightMotor.set(this.targetRightMotorSpeed * neoMotorSpeedReductionFactor);
                }
            }
        }
        // if not in speed mode we assume the caller wants smart motion
        else {

            boolean isLeftWithinTolerance =  this.isLeftClimberWithinTolerance(Constants.climberStandardToleranceInches);
            boolean isRightWithinTolerance =  this.isRightClimberWithinTolerance(Constants.climberStandardToleranceInches);
            movementWithinTolerance = isLeftWithinTolerance && isRightWithinTolerance;

            // Left
            if(this.leftClimberReady) {
                if(isLeftArmAtOrBelowLowStop && this.targetLeftClimberInches <= 0.0&& this.leftCorrectableCoupling.getMotorEncoderEverReset()) {
                    this.leftMotor.set(0.0);
                }
                else if(isLeftArmAtOrAboveHighStop && this.targetLeftClimberInches >= ClimberSubsystem.maximumArmHeightInches) {
                    this.leftMotor.set(0.0);
                }
                else if (isLeftWithinTolerance) {
                    this.leftMotor.set(0.0);
                }
                else if(currentLeftExtensionInInches < ClimberSubsystem.lengthClimberExtensionVeryCloseToStopInches ) {
                    double frogSpellExtensionDistance = 
                        (currentLeftExtensionInInches + this.targetLeftClimberInches) / 2;
                    leftPidController.setReference(
                        this.convertClimberArmsHeightToMotorEncoderPosition(frogSpellExtensionDistance),
                        ControlType.kSmartMotion);
                }
                else {
                    leftPidController.setReference(
                        this.convertClimberArmsHeightToMotorEncoderPosition(this.targetLeftClimberInches),
                        ControlType.kSmartMotion);
                }
            }

            // Right
            if(this.rightClimberReady) {
                if(isRightArmAtOrBelowLowStop && this.targetRightClimberInches <= 0.0 && this.rightCorrectableCoupling.getMotorEncoderEverReset()) {
                    this.rightMotor.set(0.0);
                }
                else if(isRightArmAtOrAboveHighStop && this.targetRightClimberInches >= ClimberSubsystem.maximumArmHeightInches) {
                    this.rightMotor.set(0.0);
                }
                else if (isRightWithinTolerance) {
                    this.rightMotor.set(0.0);
                }
                else if(currentRightExtensionInInches < ClimberSubsystem.lengthClimberExtensionVeryCloseToStopInches ) {
                    double frogSpellExtensionDistance = 
                        (currentRightExtensionInInches + this.targetRightClimberInches) / 2;
                    rightPidController.setReference(
                        this.convertClimberArmsHeightToMotorEncoderPosition(frogSpellExtensionDistance),
                        ControlType.kSmartMotion);
                }
                else {
                    rightPidController.setReference(
                        this.convertClimberArmsHeightToMotorEncoderPosition(this.targetRightClimberInches),
                        ControlType.kSmartMotion);
                }
            }
        }
    }

    /**
     * Method to update the new target position for both climbers
     * @param leftTargetHeightInInches - the left climber target set point in inches
     * @param rightTargetHeightInInches - the right climber target set point in inches
     */
    public void setClimberHeightsInInches(
        double leftTargetHeightInInches,
        double rightTargetHeightInInches) {
        this.inSpeedMode = false;
        this.movementWithinTolerance = false;
        this.targetLeftClimberInches = leftTargetHeightInInches;
        this.targetRightClimberInches = rightTargetHeightInInches;
    }

    /**
     * A method to set requested the climber motor speeds
     * @param leftClimberSpeed the speed to run the left climber motor at
     * @param verticalArmSpeed the speed to run the right climber motor at
     */
    public void setClimberSpeeds(double leftClimberSpeed, double rightClimberSpeed) {
        this.inSpeedMode = true;
        this.movementWithinTolerance = false;
        this.targetLeftMotorSpeed = MotorUtils.truncateValue(leftClimberSpeed, -1.0, 1.0);
        this.targetRightMotorSpeed = MotorUtils.truncateValue(rightClimberSpeed, -1.0, 1.0);
      }

    /**********************************************************************
    ************ PRIVATE METHODS
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
        if(this.leftClimberReady) {
            leftEncoder.setPosition(0.0);
        }
        if(this.rightClimberReady) {
            rightEncoder.setPosition(0.0);
        }
    }
 
    private double getLeftMotorEncoderPosition() {
        if(this.leftClimberReady) {
            return leftCorrectableCoupling.getCurrentEncoderPosition();
        }
        return Double.NaN;
    }

    private double getLeftMotorOutputSpeed() {
        if(this.leftClimberReady) {
            return leftMotor.getAppliedOutput();
        }
        return Double.NaN;
    }

    private String getLeftArmMotionDescription() {
        if(this.leftClimberReady) {
            return this.getArmMotionDescription(this.getLeftMotorOutputSpeed());
        }
        return "MISSING";
    }

    private double getRightMotorEncoderPosition() {
        if(this.rightClimberReady) {
            return rightCorrectableCoupling.getCurrentEncoderPosition();
        }
        return Double.NaN;
    }

    private double getRightMotorOutputSpeed() {
        if(this.rightClimberReady) {
            return rightMotor.getAppliedOutput();
        }
        return Double.NaN;
    }

    private String getRightArmMotionDescription() {
        if(this.rightClimberReady) {
            return this.getArmMotionDescription(this.getRightMotorOutputSpeed());
        }
        return "MISSING";
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

    // a method devoted to establishing proper startup of the climber motors
    // this method sets all of the key settings that will help in motion magic
    private void initializeMotorsSmartMotion() {

      if(motorsInitalizedForSmartMotion == false) { 
        int smartMotionSlot = 0;

        // left side
        if(this.leftClimberReady) {
            // PID coefficients
            kPLeft = 2e-4; 
            kILeft = 0;
            kDLeft = 0;
            kIzLeft = 0; 
            kFFLeft = 0.00001; 
            kMaxOutputLeft = 1; 
            kMinOutputLeft = -1;
            maxRPMLeft = Constants.neoMaximumRevolutionsPerMinute;
        
            // Smart Motion Coefficients
            maxVelLeft = maxRPMLeft * neoMotorSpeedReductionFactor; // rpm
            maxAccLeft = maxVelLeft * 2; // 1/2 second to get up to full speed
        
            leftMotor.restoreFactoryDefaults();
            leftMotor.setIdleMode(IdleMode.kBrake);
            leftMotor.setInverted(this.isLeftMotorInverted);
            leftPidController = leftMotor.getPIDController();
            leftEncoder = leftMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, (int)Constants.RevNeoEncoderTicksPerRevolution);
            leftEncoder.setPositionConversionFactor((double)Constants.RevNeoEncoderTicksPerRevolution);
    
            // set PID coefficients
            leftPidController.setP(kPLeft);
            leftPidController.setI(kILeft);
            leftPidController.setD(kDLeft);
            leftPidController.setIZone(kIzLeft);
            leftPidController.setFF(kFFLeft);
            leftPidController.setOutputRange(kMinOutputLeft, kMaxOutputLeft);
        
            leftPidController.setSmartMotionMaxVelocity(maxVelLeft, smartMotionSlot);
            leftPidController.setSmartMotionMinOutputVelocity(minVelLeft, smartMotionSlot);
            leftPidController.setSmartMotionMaxAccel(maxAccLeft, smartMotionSlot);
            leftPidController.setSmartMotionAllowedClosedLoopError(allowedErrLeft, smartMotionSlot);
        }

        // right side
        if(this.rightClimberReady) {
            // PID coefficients
            kPRight = 2e-4; 
            kIRight = 0;
            kDRight = 0;
            kIzRight = 0; 
            kFFRight = 0.00001; 
            kMaxOutputRight = 1; 
            kMinOutputRight = -1;
            maxRPMRight = Constants.neoMaximumRevolutionsPerMinute;
        
            // Smart Motion Coefficients
            maxVelRight = maxRPMRight * neoMotorSpeedReductionFactor; // rpm
            maxAccRight = maxVelRight * 2; // 1/2 second to get up to full speed

            rightMotor.restoreFactoryDefaults();
            rightMotor.setIdleMode(IdleMode.kBrake);
            rightMotor.setInverted(this.isRightMotorInverted);
            rightPidController = rightMotor.getPIDController();
            rightEncoder = rightMotor.getEncoder(SparkRelativeEncoder.Type.kHallSensor, (int)Constants.RevNeoEncoderTicksPerRevolution);
            rightEncoder.setPositionConversionFactor((double)Constants.RevNeoEncoderTicksPerRevolution);
    
            // set PID coefficients
            rightPidController.setP(kPRight);
            rightPidController.setI(kIRight);
            rightPidController.setD(kDRight);
            rightPidController.setIZone(kIzRight);
            rightPidController.setFF(kFFRight);
            rightPidController.setOutputRange(kMinOutputRight, kMaxOutputRight);
        
            rightPidController.setSmartMotionMaxVelocity(maxVelRight, smartMotionSlot);
            rightPidController.setSmartMotionMinOutputVelocity(minVelRight, smartMotionSlot);
            rightPidController.setSmartMotionMaxAccel(maxAccRight, smartMotionSlot);
            rightPidController.setSmartMotionAllowedClosedLoopError(allowedErrRight, smartMotionSlot);
        }

        this.motorsInitalizedForSmartMotion = true;
      }
    }
}