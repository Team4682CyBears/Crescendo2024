// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: ArmSubsystem.java
// Intent: Forms a stub for the prelminary named subsystem above.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.SparkMaxRelativeEncoder;
import com.revrobotics.CANSparkMax.*;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.common.*;
import frc.robot.control.InstalledHardware;

import java.util.*;

//import javax.lang.model.util.ElementScanner14;

public class ArmSubsystem extends SubsystemBase
{
    /* *********************************************************************
    CONSTANTS
    ************************************************************************/
    // expected to be < 1.0 due to encoder granularity being lower for Rev/Neo
    private static final double telescopingArmsMotorEncoderTicksPerDegree = Constants.RevNeoEncoderTicksPerRevolution / Constants.DegreesPerRevolution;
    // Discussion with Nathan on 02/08/2023 on 'angle arm' - 0.375" per hole * 15 teeth - gearbox is aprox 1:100
    private static final double verticalArmMovementInMetersPerMotorRotation = (0.009525 * 15) * (1.0 / 50.0); 
    // Discussion with Owen on 02/08/2023 on 'extension arm' - 5mm per tooth on belt 36 teeth - gearbox is aprox 1:10
    private static final double horizontalArmMovementInMetersPerMotorRotation = (0.005 * 36) * (1.0 / 20.0); 
    
    // the extension distances of the arms - in meters
    private static final double minimumVerticalArmExtensionMeters = 0.0;
    private static final double maximumVerticalArmExtensionMeters = Units.inchesToMeters(7.75); // 8 inches = 0.2032 meters;
    private static final double toleranceVerticalArmExtensionMeters = 0.001;
    private static final double minimumHorizontalArmExtensionMeters = 0.0;
    private static final double maximumHorizontalArmExtensionMeters = Units.inchesToMeters(70.0 - 40.25); // 70.0 - 40.25 = 30.125 inches = 0.7652 meters;
    private static final double toleranceHorizontalArmExtensionMeters = 0.003;

    private static final double verticalArmBottomSensorPlacementAlongExtensionMeters = Units.inchesToMeters(0.0);
    private static final double verticalArmMiddleSensorPlacementAlongExtensionMeters = Units.inchesToMeters(7.5);
    private static final double horizontalArmSensorPlacementAlongExtensionMeters = Units.inchesToMeters(0.0);

    // the various geometry aspects of the arm setup // 
    private static final double lengthFloorToHorizontalArmPivotMeters = Units.inchesToMeters(3.0); // 3 inches
    private static final double lengthFloorToVerticalArmPivotMeters = Units.inchesToMeters(3.125); // 3.125 inches
    private static final double lengthBasePinDistanceBetweewnHorizontalAndVerticalArmsMeters = Units.inchesToMeters(22.75); // 22.75 inches
    private static final double lengthHorizontalArmPinDistanceMeters = Units.inchesToMeters(16.5); // 16.5 inches
    private static final double lengthMinimumVerticalArmMeters = Units.inchesToMeters(14.0); // 14 inches
    private static final double lengthMaximumVerticalArmMeters = lengthMinimumVerticalArmMeters + (maximumVerticalArmExtensionMeters - minimumVerticalArmExtensionMeters);
    private static final double lengthMinimumHorizontalArmMeters = Units.inchesToMeters(40.25); // 40.25 inches
    private static final double lengthMaximumHorizontalArmMeters = lengthMinimumHorizontalArmMeters + (maximumHorizontalArmExtensionMeters - minimumHorizontalArmExtensionMeters); // ??

    private static final double lengthHorizontalArmExtensionVeryCloseToEndMeters = maximumHorizontalArmExtensionMeters - Units.inchesToMeters(2.0); // max - 2.0 inches
    private static final double lengthHorizontalArmExtensionVeryCloseToStopMeters = Units.inchesToMeters(2.0); // 2.0 inches
    private static final double lengthVerticalArmExtensionVeryCloseToPucksMeters = maximumVerticalArmExtensionMeters - Units.inchesToMeters(1.0); // max - 1.0 inch
    private static final double lengthVerticalArmExtensionVeryCloseToStopMeters = Units.inchesToMeters(1.0); // 1.0 inch
    private static final double neoMotorSpeedReductionFactorVeryCloseToStop = 0.25; 

    private static final double verticalArmSensorResetRetractSpeed = -0.7;
    private static final double verticalArmSensorResetExtendSpeed = 1.0;
    private static final double horizontalArmSensorResetRetractSpeed = -0.7;
    private static final double horizontalArmSensorResetExtendSpeed = 1.0;

    // TODO - use something less than 1.0 for testing
    private static final double neoMotorSpeedReductionFactor = 1.0;

    /* *********************************************************************
    MEMBERS
    ************************************************************************/
    // two matched motors - one for each climber side
    private CANSparkMax verticalMotor = new CANSparkMax(Constants.VerticalArmDriveMotorCanId, MotorType.kBrushless);
    private SparkMaxPIDController verticalPidController;
    private RelativeEncoder verticalEncoder;
    private CANSparkMax horizontalMotor = new CANSparkMax(Constants.HorizontalArmDriveMotorCanId, MotorType.kBrushless);
    private SparkMaxPIDController horizontalPidController;
    private RelativeEncoder horizontalEncoder;
    private double kPHorizontal, kIHorizontal, kDHorizontal, kIzHorizontal, kFFHorizontal, kMaxOutputHorizontal, kMinOutputHorizontal, maxRPMHorizontal, maxVelHorizontal, minVelHorizontal, maxAccHorizontal, allowedErrHorizontal;
    private double kPVertical, kIVertical, kDVertical, kIzVertical, kFFVertical, kMaxOutputVertical, kMinOutputVertical, maxRPMVertical, maxVelVertical, minVelVertical, maxAccVertical, allowedErrVertical;
    private boolean motorsInitalizedForSmartMotion = false;

    private DigitalInput verticalArmBottomMageneticSensor = null;
    private DigitalInput verticalArmMiddleMageneticSensor = null;
    private DigitalInput horizontalArmMageneticSensor = null;

    private boolean isHorizontalMotorInverted = false;
    private boolean isVerticalMotorInverted = true;

    private boolean inSpeedMode = true;
    private boolean movementWithinTolerance = false;
    private double requestedHorizontalMotorSpeed = 0.0;
    private double requestedVerticalMotorSpeed = 0.0;
    private double requestedHorizontalArmExtension = 0.0;
    private double requestedVerticalArmExtension = 0.0;

    CorrectableEncoderRevNeoPlusDigitalIoPort verticalArmBottomCorrectableEncoder = null;
    CorrectableEncoderRevNeoPlusDigitalIoPort verticalArmMiddleCorrectableEncoder = null;
    CorrectableEncoderRevNeoPlusDigitalIoPort horizontalArmCorrectableEncoder = null;

    /* *********************************************************************
    CONSTRUCTORS
    ************************************************************************/

    /**
    * constructor for TelescopingArms subsystem
    */
    public ArmSubsystem() {

      // init smart motion and set positions if mag sensors are set
      this.initializeMotorsSmartMotion();

      if(InstalledHardware.verticalArmBottomSensorInstalled) {
        verticalArmBottomMageneticSensor = new DigitalInput(Constants.VirticalArmBottomMagneticSensor);
        verticalArmBottomCorrectableEncoder = new CorrectableEncoderRevNeoPlusDigitalIoPort(
          verticalEncoder,
          verticalArmBottomMageneticSensor,
          ArmSubsystem.convertVerticalArmExtensionFromMetersToTicks(ArmSubsystem.verticalArmBottomSensorPlacementAlongExtensionMeters),
          ArmSubsystem.convertVerticalArmExtensionFromMetersToTicks(ArmSubsystem.minimumVerticalArmExtensionMeters - 0.05), // assume below the reference zero point by 5 cm (~2 inches)
          ArmSubsystem.convertVerticalArmExtensionFromMetersToTicks(ArmSubsystem.maximumVerticalArmExtensionMeters + 0.05)); // assume above the max travel by 5 cm (~2 inches)
      }

      if(InstalledHardware.verticalArmMiddleSensorInstalled) {
        verticalArmMiddleMageneticSensor = new DigitalInput(Constants.VirticalArmMiddleMagneticSensor);
        verticalArmMiddleCorrectableEncoder = new CorrectableEncoderRevNeoPlusDigitalIoPort(
          verticalEncoder,
          verticalArmMiddleMageneticSensor,
          ArmSubsystem.convertVerticalArmExtensionFromMetersToTicks(ArmSubsystem.verticalArmMiddleSensorPlacementAlongExtensionMeters),
          ArmSubsystem.convertVerticalArmExtensionFromMetersToTicks(ArmSubsystem.minimumVerticalArmExtensionMeters - 0.05), // assume below the reference zero point by 5 cm (~2 inches)
          ArmSubsystem.convertVerticalArmExtensionFromMetersToTicks(ArmSubsystem.maximumVerticalArmExtensionMeters + 0.05)); // assume above the max travel by 5 cm (~2 inches)
      }

      if(InstalledHardware.horizontalArmSensorInstalled) {
        horizontalArmMageneticSensor = new DigitalInput(Constants.HorizontalArmMagneticSensor);
        horizontalArmCorrectableEncoder = new CorrectableEncoderRevNeoPlusDigitalIoPort(
          horizontalEncoder,
          horizontalArmMageneticSensor,
          ArmSubsystem.convertHorizontalArmExtensionFromMetersToTicks(ArmSubsystem.horizontalArmSensorPlacementAlongExtensionMeters),
          ArmSubsystem.convertHorizontalArmExtensionFromMetersToTicks(ArmSubsystem.minimumHorizontalArmExtensionMeters - 0.05), // assume below the reference zero point by 5 cm (~2 inches)
          ArmSubsystem.convertHorizontalArmExtensionFromMetersToTicks(ArmSubsystem.maximumHorizontalArmExtensionMeters + 0.05)); // assume above the max travel by 5 cm (~2 inches)
      }

      CommandScheduler.getInstance().registerSubsystem(this);
    }

    /* *********************************************************************
    PUBLIC METHODS
    ************************************************************************/
    /**
     * A method to set requested the arms motor speeds
     * @param horizontalArmSpeed the speed to run the horizontal arm motor at
     * @param verticalArmSpeed the speed to run the vertical arm motor at
     */
    public void setArmSpeeds(double horizontalArmSpeed, double verticalArmSpeed) {
      this.inSpeedMode = true;
      this.movementWithinTolerance = false;
      this.requestedHorizontalMotorSpeed = MotorUtils.truncateValue(horizontalArmSpeed, -1.0, 1.0);
      this.requestedVerticalMotorSpeed = MotorUtils.truncateValue(verticalArmSpeed, -1.0, 1.0);
    }

    /**
     * A method to set requested the arms motor extension distance
     * @param horizontalArmExtension the distance to extend the vertical arm to
     * @param verticalArmExtension the distance to extend the vertical arm to
     */
    public void setArmExtensions(double horizontalArmExtension, double verticalArmExtension) {
      this.inSpeedMode = false;
      this.movementWithinTolerance = false;
      this.requestedHorizontalArmExtension = MotorUtils.truncateValue(horizontalArmExtension, minimumHorizontalArmExtensionMeters, maximumHorizontalArmExtensionMeters);
      this.requestedVerticalArmExtension = MotorUtils.truncateValue(verticalArmExtension, minimumVerticalArmExtensionMeters, maximumVerticalArmExtensionMeters);
    }
    
    /**
     * A method to set requested the arms motor extension distance
     * @param yPointMeters the y aspect of the arm from the primary arm pivot center in meters
     * @param zPointMeters the z aspect of the arm above the level playing floor in meters
     * @return true if the position is valid and was set, otherwise false
     */
    public boolean setArmToPointInSpace(double yPointMeters, double zPointMeters) {
      
      double requestedAngle = Math.atan(zPointMeters/yPointMeters);
      double requestedHorizontalArmLength = yPointMeters/Math.cos(requestedAngle);
      double requestedVerticalArmLength = 
        Math.sqrt(
          lengthHorizontalArmPinDistanceMeters * lengthHorizontalArmPinDistanceMeters +
          lengthBasePinDistanceBetweewnHorizontalAndVerticalArmsMeters * lengthBasePinDistanceBetweewnHorizontalAndVerticalArmsMeters -
          (2 *lengthHorizontalArmPinDistanceMeters * lengthBasePinDistanceBetweewnHorizontalAndVerticalArmsMeters * Math.cos(requestedAngle)));

      double requestedHorizontalArmExtensionMeters = requestedHorizontalArmLength - lengthMinimumHorizontalArmMeters;
      double requestedVerticalArmExtensionMeters = requestedVerticalArmLength - lengthMinimumVerticalArmMeters;

      boolean armPointInSpaceValid = false;
      if( requestedHorizontalArmExtensionMeters >= minimumHorizontalArmExtensionMeters &&
        requestedHorizontalArmExtensionMeters <= maximumHorizontalArmExtensionMeters &&
        requestedVerticalArmExtensionMeters >= minimumVerticalArmExtensionMeters &&
        requestedVerticalArmExtensionMeters <= maximumVerticalArmExtensionMeters) {
          armPointInSpaceValid = true;
          this.setArmExtensions(requestedHorizontalArmExtensionMeters, requestedVerticalArmExtensionMeters);
      }
      else {
        System.out.println("!!!INVALID POSITION REQUESTED!!!");
        System.out.println("angle radians = " + requestedAngle +
        "\nhorizontal len = " + requestedHorizontalArmLength +
        "\nvertical len = " + requestedVerticalArmLength +
        "\nhorizontal extension = " + requestedHorizontalArmExtensionMeters +
        "\nvertical extension = " + requestedVerticalArmExtensionMeters);
      }
      return armPointInSpaceValid;
    }

    /**
     * Method to help indicate when a requested movement is complete
     * @return true when the arms have arrived at their extension distances, else false
     */
    public boolean isRequestedArmMovementComplete() {
      return this.inSpeedMode == false && this.movementWithinTolerance;
    }

    /**
     * Method to confirm both arms have found their sensor reset positions
     * @return true if both arms have moved past their sensor reset positions
     */
    public boolean haveArmsFoundSensorReset(){
      return ( InstalledHardware.horizontalArmSensorInstalled && this.horizontalArmCorrectableEncoder.getMotorEncoderEverReset() ) &&
      (
        (InstalledHardware.verticalArmBottomSensorInstalled && this.verticalArmBottomCorrectableEncoder.getMotorEncoderEverReset()) || 
        (InstalledHardware.verticalArmMiddleSensorInstalled && this.verticalArmMiddleCorrectableEncoder.getMotorEncoderEverReset())
      );
    }

    /**
     * Method to move both arms toward the sensor reset positions of each arm
     * will automatically stop the arm movements when sensor reset has occurred
     */
    public void moveArmsToSensorReset() {
      
      // get ready for the arm drive speeds
      double horizontalSpeed = 0.0;
      double verticalSpeed = 0.0;

      // first horizontal arm
      if(InstalledHardware.horizontalArmSensorInstalled && // the sensor must be present to ever get any horizontal speed other than zero
         this.horizontalArmCorrectableEncoder.getMotorEncoderEverReset() == false) { // when the motor encoder has been reset, no need to move the arm
          // extend if the sensor is currently triggered
          // retract if the sensor is NOT currently triggered
          horizontalSpeed =
            (this.horizontalArmMageneticSensor.get() == false) ? // sensor light is illuminated when .get returns false
            ArmSubsystem.horizontalArmSensorResetExtendSpeed :
            ArmSubsystem.horizontalArmSensorResetRetractSpeed;
      }

      // second vertical arm
      if(InstalledHardware.verticalArmMiddleSensorInstalled && // sensor must be present to get any speed
         this.verticalArmMiddleCorrectableEncoder.getMotorEncoderEverReset() == false) { // when the motor encoder has been reset, no need to move the arm
          // extend if the sensor is currently triggered
          // retract if the sensor is NOT currently triggered
          verticalSpeed =
            (this.verticalArmMiddleMageneticSensor.get() == false) ? // sensor light is illuminated when .get returns false
            ArmSubsystem.verticalArmSensorResetExtendSpeed :
            ArmSubsystem.verticalArmSensorResetRetractSpeed;
      }
      else if(InstalledHardware.verticalArmBottomSensorInstalled && // sensor must be present to get any speed
        this.verticalArmBottomCorrectableEncoder.getMotorEncoderEverReset() == false) { // when the motor encoder has been reset, no need to move the arm
          // extend if the sensor is currently triggered
          // retract if the sensor is NOT currently triggered
          verticalSpeed =
            (this.verticalArmBottomMageneticSensor.get() == false) ? // sensor light is illuminated when .get returns false
            ArmSubsystem.verticalArmSensorResetExtendSpeed :
            ArmSubsystem.verticalArmSensorResetRetractSpeed;
      }

      this.setArmSpeeds(horizontalSpeed, verticalSpeed);
    }
    
    /**
     * A method to handle periodic processing
     */
    @Override
    public void periodic() {

      // confirm that the smart motion is setup - no-op after it is setup first time
      this.initializeMotorsSmartMotion();
      this.doTelemetry();      

      // determine if the movement is in the stop range
      // stop range implies any of the following:
      // magnetic sensor triggered 
      // arm deployed >= limit (e.g., maximumXXXArmExtensionMeters)
      double currentHorizontalExtensionInMeters = this.getCurrentHorizontalArmExtensionInMeters();
      boolean isHorizontalArmAtOrBelowLowStop = (currentHorizontalExtensionInMeters <= 0.0);
      boolean isHorizontalArmAtOrAboveHighStop = currentHorizontalExtensionInMeters >= maximumHorizontalArmExtensionMeters;
      double currentVerticalExtensionInMeters = this.getCurrentVerticalArmExtensionInMeters();
      boolean isVerticalArmAtOrBelowLowStop = (currentVerticalExtensionInMeters <= 0.0);
      boolean isVerticalArmAtOrAboveHighStop = currentVerticalExtensionInMeters >= maximumVerticalArmExtensionMeters;

      // if we are in speed mode always set motor speeds using motor set
      if(this.inSpeedMode) {

        // Horizontal
        if(isHorizontalArmAtOrBelowLowStop && this.requestedHorizontalMotorSpeed < 0.0) {
          this.horizontalMotor.set(0.0);
        }
        else if(isHorizontalArmAtOrAboveHighStop && this.requestedHorizontalMotorSpeed > 0.0) {
          this.horizontalMotor.set(0.0);
        }
        // we are slapping the sensor too hard we need to figure out how to slow down before we smack it
        else if(
          (currentHorizontalExtensionInMeters < lengthHorizontalArmExtensionVeryCloseToStopMeters &&  this.requestedHorizontalMotorSpeed < 0.0 ) ||
          (currentHorizontalExtensionInMeters > lengthHorizontalArmExtensionVeryCloseToEndMeters &&  this.requestedHorizontalMotorSpeed > 0.0 )) {
          this.horizontalMotor.set(this.requestedHorizontalMotorSpeed * neoMotorSpeedReductionFactorVeryCloseToStop);
        }
        else {
          this.horizontalMotor.set(this.requestedHorizontalMotorSpeed * neoMotorSpeedReductionFactor);
        }
        
        // Vertical
        if(isVerticalArmAtOrBelowLowStop && this.requestedVerticalMotorSpeed < 0.0) {
          this.verticalMotor.set(0.0);
        }
        else if(isVerticalArmAtOrAboveHighStop && this.requestedVerticalMotorSpeed > 0.0) {
          this.verticalMotor.set(0.0);
        }
        // we are nearing puck-zone or bottom sometimes too fast
        else if(
          (currentVerticalExtensionInMeters > lengthVerticalArmExtensionVeryCloseToPucksMeters &&  this.requestedVerticalMotorSpeed > 0.0 ) || 
          (currentVerticalExtensionInMeters < lengthVerticalArmExtensionVeryCloseToStopMeters &&  this.requestedVerticalMotorSpeed < 0.0 )) {
          this.verticalMotor.set(this.requestedVerticalMotorSpeed * neoMotorSpeedReductionFactorVeryCloseToStop);
        }
        else {
          this.verticalMotor.set(this.requestedVerticalMotorSpeed * neoMotorSpeedReductionFactor);
        }

      }
      // if not in speed mode we assume the caller wants smart motion
      else {

        boolean isHorizontalWithinTolerance =  (Math.abs(currentHorizontalExtensionInMeters - this.requestedHorizontalArmExtension) <= toleranceHorizontalArmExtensionMeters);
        boolean isVerticalWithinTolerance =  (Math.abs(currentVerticalExtensionInMeters - this.requestedVerticalArmExtension) <= toleranceVerticalArmExtensionMeters);
        movementWithinTolerance = isHorizontalWithinTolerance && isVerticalWithinTolerance;

        // Horizontal
        if(isHorizontalArmAtOrBelowLowStop && this.requestedHorizontalArmExtension <= 0.0) {
          this.horizontalMotor.set(0.0);
        }
        else if(isHorizontalArmAtOrAboveHighStop && this.requestedHorizontalArmExtension >= maximumHorizontalArmExtensionMeters) {
          this.horizontalMotor.set(0.0);
        }
        else if (isHorizontalWithinTolerance) {
          this.horizontalMotor.set(0.0);
        }
        // we are slapping the sensor too hard we need to figure out how to slow down before we smack it
        else if(currentHorizontalExtensionInMeters < lengthHorizontalArmExtensionVeryCloseToStopMeters ) {
          double frogSpellExtensionDistance = 
            (currentHorizontalExtensionInMeters + this.requestedHorizontalArmExtension) / 2;
          horizontalPidController.setReference(
            ArmSubsystem.convertHorizontalArmExtensionFromMetersToTicks(frogSpellExtensionDistance),
            ControlType.kSmartMotion);
        }
        else {
          horizontalPidController.setReference(
            ArmSubsystem.convertHorizontalArmExtensionFromMetersToTicks(this.requestedHorizontalArmExtension),
            ControlType.kSmartMotion);
        }

        // Vertical
        if(isVerticalArmAtOrBelowLowStop && this.requestedVerticalArmExtension <= 0.0) {
          this.verticalMotor.set(0.0);
        }
        else if(isVerticalArmAtOrAboveHighStop && this.requestedVerticalArmExtension >= maximumVerticalArmExtensionMeters) {
          this.verticalMotor.set(0.0);
        }
        else if (isVerticalWithinTolerance) {
          this.verticalMotor.set(0.0);
        }
        // we are nearing puck-zone sometimes too fast
        else if(currentVerticalExtensionInMeters > lengthVerticalArmExtensionVeryCloseToPucksMeters) {
          double frogSpellExtensionDistance = 
            (currentVerticalExtensionInMeters + this.requestedVerticalArmExtension) / 2;
          verticalPidController.setReference(
            ArmSubsystem.convertVerticalArmExtensionFromMetersToTicks(frogSpellExtensionDistance),
            ControlType.kSmartMotion);
        }
        // we are nearing bottom stop zone sometimes too fast
        else if(currentVerticalExtensionInMeters < lengthVerticalArmExtensionVeryCloseToStopMeters) {
          double frogSpellExtensionDistance = 
            (currentVerticalExtensionInMeters + this.requestedVerticalArmExtension) / 2;
          verticalPidController.setReference(
            ArmSubsystem.convertVerticalArmExtensionFromMetersToTicks(frogSpellExtensionDistance),
            ControlType.kSmartMotion);
        }
        else {
          verticalPidController.setReference(
            ArmSubsystem.convertVerticalArmExtensionFromMetersToTicks(this.requestedVerticalArmExtension),
            ControlType.kSmartMotion);
        }

      }
    }

    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }

    /* *********************************************************************
    PRIVATE METHODS
    ************************************************************************/
    /**
     * Convert the horizontal arm extension from ticks to meters
     * @param targetPositionTicks - the arms extension distance in ticks
     * @return the distance in meters the arm extension is epected for coresponding ticks
     */
    private static double convertHorizontalArmExtensionFromTicksToMeters(double targetPositionTicks) {
      return targetPositionTicks / Constants.RevNeoEncoderTicksPerRevolution * horizontalArmMovementInMetersPerMotorRotation;
    }

    /**
     * Convert the vertical arm extension from ticks to meters
     * @param targetPositionTicks - the arms extension distance in ticks
     * @return the distance in meters the arm extension is epected for coresponding ticks
     */
    private static double convertVerticalArmExtensionFromTicksToMeters(double targetPositionTicks) {
      return targetPositionTicks / Constants.RevNeoEncoderTicksPerRevolution * verticalArmMovementInMetersPerMotorRotation;
    }

    /**
     * Convert the horizontal arm extension from meters to ticks
     * @param targetPositionMeters - the arms extension distance in meters
     * @return the distance in motor encoder ticks epected for the arm extension in meters
     */
    private static double convertHorizontalArmExtensionFromMetersToTicks(double extensionInMeters) {
      return extensionInMeters * Constants.RevNeoEncoderTicksPerRevolution / horizontalArmMovementInMetersPerMotorRotation;
    }

    /**
     * Convert the vertical arm extension from meters to ticks
     * @param targetPositionMeters - the arms extension distance in meters
     * @return the distance in motor encoder ticks epected for the arm extension in meters
     */
    private static double convertVerticalArmExtensionFromMetersToTicks(double extensionInMeters) {
      return extensionInMeters * Constants.RevNeoEncoderTicksPerRevolution / verticalArmMovementInMetersPerMotorRotation;
    }

    /**
     * Telemetry to shuffleboard
     */
    private void doTelemetry() {
      if(InstalledHardware.horizontalArmSensorInstalled){
        SmartDashboard.putBoolean("HorizontalArmSensor",  this.horizontalArmMageneticSensor.get());
        SmartDashboard.putBoolean("HorizontalArmSensorEncoderEverReset", this.horizontalArmCorrectableEncoder.getMotorEncoderEverReset());
      }
      if(InstalledHardware.verticalArmBottomSensorInstalled){
        SmartDashboard.putBoolean("VerticalArmBottomSensor", this.verticalArmBottomMageneticSensor.get());
        SmartDashboard.putBoolean("VerticalArmBottomSensorEncoderEverReset", this.verticalArmBottomCorrectableEncoder.getMotorEncoderEverReset());
      }
      if(InstalledHardware.verticalArmMiddleSensorInstalled) {
        SmartDashboard.putBoolean("VerticalArmMiddleSensor",  this.verticalArmMiddleMageneticSensor.get());
        SmartDashboard.putBoolean("VerticalArmMiddleSensorEncoderEverReset", this.verticalArmMiddleCorrectableEncoder.getMotorEncoderEverReset());
      }
      SmartDashboard.putNumber("ExtensionHorizontalArmMeters", this.getCurrentHorizontalArmExtensionInMeters());
      SmartDashboard.putNumber("ExtensionVerticalArmMeters", this.getCurrentVerticalArmExtensionInMeters());

      // removing for now as currently unnecessary
      /* 
      SmartDashboard.putBoolean("haveArmsFoundSensorReset", this.haveArmsFoundSensorReset());
      SmartDashboard.putNumber("HorizontalArmMotorTicks", this.horizontalEncoder.getPosition());
      SmartDashboard.putNumber("VerticalArmMotorTicks", this.verticalEncoder.getPosition());
      SmartDashboard.putNumber("ArmAngleRadians", this.getCurrentHorizontalArmAngleRadians());
      SmartDashboard.putNumber("ArmAngleDegrees", this.getCurrentHorizontalArmAngleDegrees());
      SmartDashboard.putNumber("ArmHeightMetersZ", this.getCurrentArmsHeightInMeters());
      SmartDashboard.putNumber("ArmDistanceMetersY", this.getCurrentArmsDistanceInMeters());
      */
    }

    /**
     * A method to return the current horizontal arms extension in meters
     * @return the distance in meters the arm is expected to be deployed based on current motor encoder values
     */
    private double getCurrentHorizontalArmExtensionInMeters() {
      return InstalledHardware.horizontalArmSensorInstalled ?
        ArmSubsystem.convertHorizontalArmExtensionFromTicksToMeters(this.horizontalArmCorrectableEncoder.getCurrentEncoderPosition()) :
        ArmSubsystem.convertHorizontalArmExtensionFromTicksToMeters(this.horizontalEncoder.getPosition());
    }

    /**
     * A method to return the current vertical arms extension in meters
     * @return the distance in meters the arm is expected to be deployed based on current motor encoder values
     */
    private double getCurrentVerticalArmExtensionInMeters() {

      // throw low on floor - just need to make sure encoder gets reset in event at low sensor
      if(InstalledHardware.verticalArmBottomSensorInstalled) {
        this.verticalArmBottomCorrectableEncoder.getCurrentEncoderPosition();
      }

      double verticalArmExtension = InstalledHardware.verticalArmMiddleSensorInstalled ?
        ArmSubsystem.convertVerticalArmExtensionFromTicksToMeters(this.verticalArmMiddleCorrectableEncoder.getCurrentEncoderPosition()) :
        ArmSubsystem.convertVerticalArmExtensionFromTicksToMeters(this.verticalEncoder.getPosition());

      return verticalArmExtension;
    }

    /**
     * A method to return the current horizontal arm angle measured from floor to arm centerline
     * @return the angle
     */
    private double getCurrentHorizontalArmAngleRadians() {
      // need to use arc cos equation for angle given all three sides
      double a = lengthMinimumVerticalArmMeters + this.getCurrentVerticalArmExtensionInMeters();
      double b = lengthHorizontalArmPinDistanceMeters;
      double c = lengthBasePinDistanceBetweewnHorizontalAndVerticalArmsMeters;
      return Math.acos((Math.pow(b, 2) + Math.pow(c, 2) - Math.pow(a, 2))/(2 * b * c));
    }

    /**
     * A method to return the current horizontal arm angle measured from floor to arm centerline
     * @return the angle
     */
    private double getCurrentHorizontalArmAngleDegrees() {
      return Units.radiansToDegrees(this.getCurrentHorizontalArmAngleRadians());
    }

    /**
     * A method to return the current arms Z height in meters from the floor
     * @return the height in meters from the floor to the arm tip
     */
    private double getCurrentArmsHeightInMeters() {
      return lengthFloorToHorizontalArmPivotMeters + (Math.sin(this.getCurrentHorizontalArmAngleRadians()) * (lengthMinimumHorizontalArmMeters + this.getCurrentHorizontalArmExtensionInMeters()));
    }

    /**
     * A method to return the current arms Y distance in meters from the floor
     * @return the height in meters from the floor to the arm tip
     */
    private double getCurrentArmsDistanceInMeters() {
      return Math.cos(this.getCurrentHorizontalArmAngleRadians()) * (lengthMinimumHorizontalArmMeters + this.getCurrentHorizontalArmExtensionInMeters());
    }

    // a method devoted to establishing proper startup of the jaws motors
    // this method sets all of the key settings that will help in motion magic
    private void initializeMotorsSmartMotion() {
      if(motorsInitalizedForSmartMotion == false) { 
        // PID coefficients
        kPHorizontal = 2e-4; 
        kIHorizontal = 0;
        kDHorizontal = 0;
        kIzHorizontal = 0; 
        kFFHorizontal = 0.00001; 
        kMaxOutputHorizontal = 1; 
        kMinOutputHorizontal = -1;
        maxRPMHorizontal = Constants.neoMaximumRevolutionsPerMinute;
        int smartMotionSlot = 0;
    
        // Smart Motion Coefficients
        maxVelHorizontal = maxRPMHorizontal * neoMotorSpeedReductionFactor; // rpm
        maxAccHorizontal = maxVelHorizontal * 2; // 1/2 second to get up to full speed
      
        horizontalMotor.restoreFactoryDefaults();
        horizontalMotor.setIdleMode(IdleMode.kBrake);
        horizontalMotor.setInverted(this.isHorizontalMotorInverted);
        horizontalPidController = horizontalMotor.getPIDController();
        horizontalEncoder = horizontalMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, (int)Constants.RevNeoEncoderTicksPerRevolution);
        horizontalEncoder.setPositionConversionFactor((double)Constants.RevNeoEncoderTicksPerRevolution);
   
        // set PID coefficients
        horizontalPidController.setP(kPHorizontal);
        horizontalPidController.setI(kIHorizontal);
        horizontalPidController.setD(kDHorizontal);
        horizontalPidController.setIZone(kIzHorizontal);
        horizontalPidController.setFF(kFFHorizontal);
        horizontalPidController.setOutputRange(kMinOutputHorizontal, kMaxOutputHorizontal);
    
        horizontalPidController.setSmartMotionMaxVelocity(maxVelHorizontal, smartMotionSlot);
        horizontalPidController.setSmartMotionMinOutputVelocity(minVelHorizontal, smartMotionSlot);
        horizontalPidController.setSmartMotionMaxAccel(maxAccHorizontal, smartMotionSlot);
        horizontalPidController.setSmartMotionAllowedClosedLoopError(allowedErrHorizontal, smartMotionSlot);

        // PID coefficients
        kPVertical = 2e-4; 
        kIVertical = 0;
        kDVertical = 0;
        kIzVertical = 0; 
        kFFVertical = 0.00001; 
        kMaxOutputVertical = 1; 
        kMinOutputVertical = -1;
        maxRPMVertical = Constants.neoMaximumRevolutionsPerMinute;
    
        // Smart Motion Coefficients
        maxVelVertical = maxRPMVertical * neoMotorSpeedReductionFactor; // rpm
        maxAccVertical = maxVelVertical * 2; // 1/2 second to get up to full speed

        verticalMotor.restoreFactoryDefaults();
        verticalMotor.setIdleMode(IdleMode.kBrake);
        verticalMotor.setInverted(this.isVerticalMotorInverted);
        verticalPidController = verticalMotor.getPIDController();
        verticalEncoder = verticalMotor.getEncoder(SparkMaxRelativeEncoder.Type.kHallSensor, (int)Constants.RevNeoEncoderTicksPerRevolution);
        verticalEncoder.setPositionConversionFactor((double)Constants.RevNeoEncoderTicksPerRevolution);
   
        // set PID coefficients
        verticalPidController.setP(kPVertical);
        verticalPidController.setI(kIVertical);
        verticalPidController.setD(kDVertical);
        verticalPidController.setIZone(kIzVertical);
        verticalPidController.setFF(kFFVertical);
        verticalPidController.setOutputRange(kMinOutputVertical, kMaxOutputVertical);
    
        verticalPidController.setSmartMotionMaxVelocity(maxVelVertical, smartMotionSlot);
        verticalPidController.setSmartMotionMinOutputVelocity(minVelVertical, smartMotionSlot);
        verticalPidController.setSmartMotionMaxAccel(maxAccVertical, smartMotionSlot);
        verticalPidController.setSmartMotionAllowedClosedLoopError(allowedErrVertical, smartMotionSlot);

        this.motorsInitalizedForSmartMotion = true;
      }
    }
}
