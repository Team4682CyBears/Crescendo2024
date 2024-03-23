// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: Constants.java
// Intent: Forms key constants required for this robot.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.control;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import frc.robot.common.DrivetrainSwerveConfig;
import frc.robot.swerveHelpers.WcpModuleConfigurations;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public final class Constants {

    public final static double DriveVoltageScalar = 1.0;

    //////////////////// TED DRIVETRAIN ////////////////////
    public static final DrivetrainSwerveConfig tedDrivertainConfig = new DrivetrainSwerveConfig(
        Units.inchesToMeters(23.25), 
        Units.inchesToMeters(22.75), 
        WcpModuleConfigurations.TED,
        // SUBTRACT the values you find in shuffleboard
        Math.toRadians(-215.15 - 178.76 - 180.0 - 358.7), // FRONT LEFT
        Math.toRadians(-180.61 - 95.27 - 358.6 -169.8 - 150.59), // FRONT RIGHT 
        Math.toRadians(-191.33 - 257.52 -357.3  - 3.6 - 355.67), // BACK LEFT
        Math.toRadians(-58.35 - 177.27 - 180.0 - 2.0 - 358.50)); // BACK RIGHT

    //////////////////// BABYBEAR DRIVETRAIN ////////////////////
    public static final DrivetrainSwerveConfig babybearDrivetrainConfig = new DrivetrainSwerveConfig (
        Units.inchesToMeters(17.179), 
        Units.inchesToMeters(17.179),
        WcpModuleConfigurations.BABYBEAR,
        Math.toRadians(281.8 + 3.8 + 173.5), // FRONT LEFT
        Math.toRadians(327.9 + 3.6 - 0.3 - 173.41), // FRONT RIGHT
        Math.toRadians(209.1 + 2.5 - 7.9), // BACK LEFT
        Math.toRadians(49.0 + 356.6 + 0.5)); // BACK RIGHT

    //////////////////// COMMON DRIVETRAIN ////////////////////
    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1; 
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 2; 
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 3; 

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 4; 
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 5; 
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 6; 

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 7; 
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 8; 
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 9; 

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 10; 
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 11; 
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 12; 

    // *****************************************************************
    // standard stuff constants - motors rotation, etc.
    public static final double DegreesPerRevolution = 360.0;
    // NEO maximum RPM 
    public static final double neoMaximumRevolutionsPerMinute = 5676;
    // NEO 550 maximum RPM - see: https://www.revrobotics.com/rev-21-1651/#:~:text=The%20following%20specifications%20for%20the%20NEO%20550%20Brushless,Motor%20Kv%3A%20917%20Kv%20Free%20Speed%3A%2011000%20RPM
    public static final double neoFiveFiveZeroMaximumRevolutionsPerMinute = 11000;
    // this uses the halls effect sensor when plugged into the spark max
    // see: https://www.revrobotics.com/rev-21-1650/ where it says "42 counts per rev."
    public static final double RevNeoEncoderTicksPerRevolution = 42;
    // CTRE motor constants
    public static final double talonMaximumRevolutionsPerMinute = 6380;
    public static final double CtreTalonFx500EncoderTicksPerRevolution = 2048; 

    // *****************************************************************
    // input device constants
    public static final int portDriverController = 0;
    public static final int portCoDriverController = 1;

    // *****************************************************************
    // navx isLevel tolerence in degrees
    public static final double navxTolDegrees = 4;
    // ************************************
    // trajectory constants
    public static final double TrajectoryMaxAcceleration = 1;
    public static final double TrajectoryMaxVelocity = 1;
    // tolerence on trajectory locations
    // TODO test tightening up these values
    public static final Pose2d TrajectoryPoseTol = new Pose2d(0.1, 0.1, Rotation2d.fromDegrees(5));

    //*****************************************
    // Fine placement constants
    // Center of rotation for fine placement
    public static final Translation2d RobotFrontRotationalCenter = 
        InstalledHardware.tedDrivetrainInstalled ?
        new Translation2d(tedDrivertainConfig.getWheelbaseMeters()/2 + Units.inchesToMeters(10), 0.0) :
        new Translation2d(babybearDrivetrainConfig.getWheelbaseMeters()/2 + Units.inchesToMeters(10), 0.0);
    // velocity for fine placement
    public static final double FinePlacementRotationalVelocity = 0.7;

    // ******************************************************************
    // intake constants
    public static final int intakeMotorCanId = 13;
    public static final int intakeTofCanId = 14;
    // intakeSpeed is [-1.0 .. 1.0]
    public static final double intakeSpeed = -1.0;
    public static final double removeSpeed = 1.0;
    // intake will run until note is detected or this timeout has expired
    public static final double intakeTimeoutSeconds = 10.0;

    // ******************************************************************
    // feeder constants
    public static final int feederMotorCanId = 15;
    public static final int firstFeederToShooterTofCanId = 16;
    public static final int secondFeederToShooterTofCanId = 17;
    // feederSpeed is [0.0 .. 1.0]
    // it runs in one direction for the shooter 
    // and the opposite direction for the dunker/amp
    public static final double feederSpeed = 0.30;
    public static final double feederReverseSpeed = 0.10;
    // feeder will run until note is detected or this timeout has expired
    public static final double feederTimeoutSeconds = 10.0;
    public static final double feederLaunchTimeoutSecondsInAuto = .75;
    public static final double feederLaunchTimeoutSecondsInTele = 2.00;

    // *******************************************************************
    // shooter outfeed constants
    public static final int leftTalonShooterMotorCanId = 18; 
    public static InvertedValue leftTalonShooterMotorDefaultDirection = InvertedValue.CounterClockwise_Positive;  
    public static final int rightTalonShooterMotorCanId = 21; 
    public static InvertedValue rightTalonShooterMotorDefaultDirection = InvertedValue.CounterClockwise_Positive;
    // Falcon 500 max speed is 6380 RPM, max loaded speed is 5600 deteremined from testing, 
    // but 5000 is a more comfortable max
    public static final double shooterMaxRpm = 5000;
    public static final double shooterDefaultSpeedRpm = shooterMaxRpm;

    public static final double shooterSpinUpTimeoutSeconds = 5.0;
    public static final double autoShooterSpinUpTimeoutSeconds = 15.0;
    public static final double shooterShootDuration = 1.7;
    public static final double shooterSpinUpDelay = 0.8;

    // speeds for specific shooter shots
    public static final double shooterOutfeedSpeedForAngleShootFromSpeaker = 4500.0;
    public static final double shooterOutfeedSpeedForAngleShootFromNote = shooterMaxRpm; 
    public static final double shooterOutfeedSpeedForAngleShootFromStage = shooterMaxRpm;
    public static final double shooterOutfeedSpeedForAngleShootFromSourceWing = shooterMaxRpm;
    public static final double shooterOutfeedSpeedForAngleShootFromAmp = 250.0;

    // *******************************************************************
    // shooter angle constants  
    public static final int shooterLeftAngleMotorCanId = 22;
    public static final int shooterRightAngleMotorCanId = 23;
    public static final int shooterLeftAngleEncoderCanId = 24;
    // TODO depending on which side the motor is mounted, may need to invert these.
    public static InvertedValue angleLeftTalonShooterMotorDefaultDirection = InvertedValue.CounterClockwise_Positive;
    public static InvertedValue angleRightTalonShooterMotorDefaultDirection = InvertedValue.Clockwise_Positive;
    public static final double shooterAbsoluteAngleOffsetDegrees = 59.445;
    public static final double shooterStartingAngleOffsetDegrees = 20.0; 
    public static SensorDirectionValue shooterAngleSensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    public static final double shooterAngleMaxDegrees = 110;
    public static final double shooterAngleMinDegrees = 20;  
    // stow angle should be low enough to drive under the stage
    public static final double shooterAngleStowDegrees = 25; 
    public static final double shooterAngleToleranceDegrees = 0.5;
    public static final double shooterSetAngleDuration = 3.0;
    // angles increment ranges of stick input
    public static final double shooterAngleStickIncrementMagnitude = Constants.shooterAngleToleranceDegrees + 12.0;
    // angles of shooter shots
    public static final double shooterAngleShootFromSpeaker = 56.0;
    // TODO - figure out what 'shoot from note' actually means??
    public static final double shooterAngleShootFromNote = 42.0; 
    public static final double shooterAngleShootFromStage = 40.0;
    public static final double shooterAngleShootFromSideStage = 34.0;
    public static final double shooterAngleShootFromSourceWing = 22.0;
    public static final double shooterAngleShootFromAmp = Constants.shooterAngleMaxDegrees;

    // ******************************************************************
    // climber constants
    public static final int leftClimberMotorCanId = 26;
    public static final int rightClimberMotorCanId = 25;
    public static final int leftClimberSensorDioId = 0;
    public static final int rightClimberSensorDioId= 1;
    public static final double climberStandardToleranceInches = 0.25;
    public static final double climberControllerStickDeadband = 0.2;
    public static final double climberArmSensorPosition = 0.75;
    // the blind find distance represents the maximum distance the climber will retract
    // in the sequence where it attempts to find its sensor zero before deciding the sensor
    // is not present in the system and in place it creates an assumed zero position
    public static final double climberArmSensorBlindFindDistance = 1.75;
    public static final double climberArmToPositionFullDeploy = 22.75;
    public static final double climberArmToPositionFullRetract = -2.0;
    public static final double climberArmToPositionHighChain = 18.0;
    public static final double climberArmToPositionLowChain = 15.0;
    public static final double climberArmToPositionHangRobot = 10.0;
    public static final double climberArmUpDefaultSpeed = 1.0;
    public static final double climberArmDownDefaultSpeed = -1.0 * climberArmUpDefaultSpeed;

    // ******************************************************************
    // amp/dunker constants
    public static final int ampShoulderMotorCanId = 27;
    public static final int ampOuttakeMotorCanId = 28;

    // ******************************************************************
    // camera constants
    public static final int speakerBlueTagID = 7;
    public static final int spekaerRedTagID = 4;
  
    // ********************************************************************
    // Controller Constants
    public static final double rumbleTimeSeconds = 0.15;

    // ********************************************************************
    // PowerDistributionPanel Constants
    public static final int currentPowerDistributionPanelCanId = 29;
    public static final ModuleType currentPowerDistributionPanelType = ModuleType.kRev;
    public static final double overcurrentRumbleTimeSeconds = 0.25;


}