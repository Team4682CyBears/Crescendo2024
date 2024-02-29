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
import frc.robot.common.DrivetrainConfig;
import frc.robot.swerveHelpers.WcpModuleConfigurations;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public final class Constants {

    public final static double DriveVoltageScalar = 1.0;

    //////////////////// TED DRIVETRAIN ////////////////////
    public static final DrivetrainConfig tedDrivertainConfig = new DrivetrainConfig(
        Units.inchesToMeters(23.25), 
        Units.inchesToMeters(22.75), 
        WcpModuleConfigurations.TED,
        Math.toRadians(-215.15), // FRONT LEFT
        Math.toRadians(-180.61), // FRONT RIGHT 
        Math.toRadians(-191.33), // BACK LEFT
        Math.toRadians(-58.35)); // BACK RIGHT

    //////////////////// BABYBEAR DRIVETRAIN ////////////////////
    public static final DrivetrainConfig babybearDrivetrainConfig = new DrivetrainConfig (
        Units.inchesToMeters(17.179), 
        Units.inchesToMeters(17.179),
        WcpModuleConfigurations.BABYBEAR,
        Math.toRadians(281.8 + 3.8 + 173.5), // FRONT LEFT
        Math.toRadians(327.9 + 3.6 - 0.3), // FRONT RIGHT
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
    public static final int feederToShooterTofCanId = 16;
    public static final int feederToDunkerTofCanId = 17;
    // feederSpeed is [0.0 .. 1.0]
    // it runs in one direction for the shooter 
    // and the opposite direction for the dunker/amp
    public static final double feederSpeed = 0.30;
    public static final double feederReverseSpeed = 0.10;
    // feeder will run until note is detected or this timeout has expired
    public static final double feederTimeoutSeconds = 10.0;

    // *******************************************************************
    // shooter outfeed constants
    public static final int leftTalonShooterMotorCanId = 18; 
    public static InvertedValue leftTalonShooterMotorDefaultDirection = InvertedValue.CounterClockwise_Positive;  
    public static final int rightTalonShooterMotorCanId = 21; 
    public static InvertedValue rightTalonShooterMotorDefaultDirection = InvertedValue.Clockwise_Positive;  
    private static double shooterBaseRpm = talonMaximumRevolutionsPerMinute;
    public static final double shooterLeftDefaultSpeedRpm = shooterBaseRpm * 0.15;
    public static final double shooterRightDefaultSpeedRpm = shooterBaseRpm * 0.15;
    public static final double shooterSpinUpTimeoutSeconds = 5.0;
    public static final double shooterShootDuration = 5.0;
    public static final double shooterSpinUpDelay = 2.0;

    // *******************************************************************
    // shooter angle constants  
    public static final int shooterLeftAngleMotorCanId = 22;
    public static final int shooterRightAngleMotorCanId = 23;
    public static final int shooterLeftAngleEncoderCanId = 24;
    // TODO depending on which side the motor is mounted, may need to invert these.
    public static InvertedValue angleLeftTalonShooterMotorDefaultDirection = InvertedValue.CounterClockwise_Positive;
    public static InvertedValue angleRightTalonShooterMotorDefaultDirection = InvertedValue.Clockwise_Positive;
    public static final double shooterAbsoluteAngleOffsetDegrees = 58.5;
    public static final double shooterStartingAngleOffsetDegrees = 20.0; 
    public static SensorDirectionValue shooterAngleSensorDirection = SensorDirectionValue.CounterClockwise_Positive;
    public static final double shooterAngleToleranceDegrees = 0.5;
    public static final double shooterSetAngleDuration = 3.0;
    // angles increment ranges of stick input (deadband range)
    public static final double shooterControllerInputPositiveStickAngleIncrement = 0.15;
    public static final double shooterControllerInputNegativeStickAngleIncrement = -0.15;
    public static final double shooterAngleStickIncrementMagnitude = 0.5;

    // ******************************************************************
    // shooter game play constants
    // speeds for different spots on field
    public static final double shooterSpeedRpmStopped = 0;
    public static final double shooterSpeedRpmSpeakerIdle = 3000;
    public static final double shooterSpeedRpmSpeakerCloseDistance = 4000;
    public static final double shooterSpeedRpmSpeakerPodiumDistance = 5500;
    public static final double shooterSpeedRpmSpeakerRedlineDistance = 6250;
    public static final double shooterSpeedRpmAmpIdle = 200;
    public static final double shooterSpeedRpmAmpLow = 350;
    public static final double shooterSpeedRpmAmpMedium = 375;
    public static final double shooterSpeedRpmAmpHigh = 400;
    // angles for different spots on field
    public static final double shooterAngleDegreesStow = 45; 
    public static final double shooterAngleDegreesSpeakerCloseDistance = 55;
    public static final double shooterAngleDegreesSpeakerPodiumDistance = 40;
    public static final double shooterAngleDegreesSpeakerRedlineDistance = 24;
    public static final double shooterAngleDegreesAmpLow = 109.0;
    public static final double shooterAngleDegreesAmpMedium = 109.5;
    public static final double shooterAngleDegreesAmpHigh = 110.0;
    public static final double shooterAngleDegreesClimbStow = 90; 
    public static final double shooterAngleDegreesMinimum = 20;
    public static final double shooterAngleDegreesMaximum = 110;

    // ******************************************************************
    // climber constants
    public static final int leftClimberMotorCanId = 25;
    public static final int rightClimberMotorCanId = 26;
    public static final int leftClimberSensorDioId = 0;
    public static final int rightClimberSensorDioId= 1;
    public static final double climberStandardToleranceInches = 0.25;
    // increment ranges of stick input (deadband range)
    public static final double climberControllerInputPositiveStickAngleIncrement = shooterControllerInputPositiveStickAngleIncrement;
    public static final double climberControllerInputNegativeStickAngleIncrement = shooterControllerInputNegativeStickAngleIncrement;
    public static final double climberAngleStickIncrementMagnitude = shooterAngleStickIncrementMagnitude;

    // ******************************************************************
    // amp/dunker constants
    public static final int ampShoulderMotorCanId = 27;
    public static final int ampOuttakeMotorCanId = 28;
  
    // ********************************************************************
    // Controller Constants
    public static final double rumbleTimeSeconds = 0.15;

    // ********************************************************************
    // PowerDistributionPanel Constants
    public static final int currentPowerDistributionPanelCanId = 29;
    public static final ModuleType currentPowerDistributionPanelType = ModuleType.kRev;
    public static final double overcurrentRumbleTimeSeconds = 0.25;
    // TODO remove these if not needed
    public static final int EveryBotMotorPdpPortId = 8;
    public static final double EveryBotMotorMaximuCurrentAmps = 45.5;

}