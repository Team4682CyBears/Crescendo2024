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

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

public final class Constants {

    public final static double DriveVoltageScalar = 1.0;
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(23.25); 
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(22.75); 

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1; 
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 2; 
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 3; 
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = Math.toRadians(281.8 + 3.8 + 173.5);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 4; 
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 5; 
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 6; 
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = Math.toRadians(327.9 + 3.6 - 0.3); 

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 7; 
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 8; 
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 9; 
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = Math.toRadians(209.1 + 2.5 - 7.9);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 10; 
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 11; 
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 12; 
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = Math.toRadians(49.0 + 356.6 + 0.5);
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

    // *************************************** 
    // For auto constants
    public static final double snoutDepth = Units.inchesToMeters(2.75);

    //*****************************************
    // Fine placement constants
    // Center of rotation for fine placement
    public static final Translation2d RobotFrontRotationalCenter = new Translation2d(DRIVETRAIN_WHEELBASE_METERS/2 + snoutDepth + Units.inchesToMeters(10), 0.0);
    // velocity for fine placement
    public static final double FinePlacementRotationalVelocity = 0.7;

    // ******************************************************************
    // intake constants
    public static final int intakeMotorCanId = 13;
    public static final int intakeTofCanId = 14;
    // intakeSpeed is [-1.0 .. 1.0]
    public static final double intakeSpeed = -1.0;
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
    public static final double feederSpeed = 0.5;
    // feeder will run until note is detected or this timeout has expired
    public static final double feederTimeoutSeconds = 10.0;

    // *******************************************************************
    // shooter outfeed constants  
    public static final int leftTopTalonShooterMotorCanId = 18; 
    public static final int leftBottomTalonShooterMotorCanId = 19;
    public static InvertedValue leftTalonShooterMotorDefaultDirection = InvertedValue.CounterClockwise_Positive;  
    public static final int rightTopTalonShooterMotorCanId = 20; 
    public static final int rightBottomTalonShooterMotorCanId = 21; 
    public static InvertedValue rightTalonShooterMotorDefaultDirection = InvertedValue.Clockwise_Positive;  
    private static double shooterBaseRpm = 6500;
    public static final double shooterLeftDefaultSpeedRpm = shooterBaseRpm * 1.0;
    public static final double shooterRightDefaultSpeedRpm = shooterBaseRpm * 0.75;
    public static final double shooterSpinUpTimeoutSeconds = 5.0;
    public static final double shooterShootDuration = 0.5;

    // *******************************************************************
    // shooter angle constants  
    public static final int shooterLeftAngleMotorCanId = 22;
    public static final int shooterRightAngleMotorCanId = 23;
    public static final int shooterLeftAngleEncoderCanId = 24;
    // TODO depending on which side the motor is mounted, may need to invert these.
    public static InvertedValue angleLeftTalonShooterMotorDefaultDirection = InvertedValue.Clockwise_Positive;
    public static InvertedValue angleRightTalonShooterMotorDefaultDirection = InvertedValue.CounterClockwise_Positive;
    public static final double shooterAngleOffsetDegrees = -0.0;
    public static SensorDirectionValue shooterAngleSensorDirection = SensorDirectionValue.Clockwise_Positive;
    public static final double shooterAngleMaxDegrees = 90;
    public static final double shooterAngleMinDegrees = 0;  
    // stow angle should be low enough to drive under the stage
    public static final double shooterAngleStowDegrees = 45; 
    public static final double shooterAngleToleranceDegrees = 3;

    // ******************************************************************
    // climber constants
    public static final int leftClimberMotorCanId = 25;
    public static final int rightClimberMotorCanId = 26;
    public static final int leftClimberSensorDioId = 1;
    public static final int rightClimberSensorDioId= 2;

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