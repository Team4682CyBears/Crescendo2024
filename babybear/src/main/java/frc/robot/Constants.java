// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: Constants.java
// Intent: Forms key constants required for this robot.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;

public final class Constants {

    public final static double DriveVoltageScalar = 1.0;
    /**
     * The left-to-right distance between the drivetrain wheels
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_TRACKWIDTH_METERS = Units.inchesToMeters(17.179); 
    /**
     * The front-to-back distance between the drivetrain wheels.
     *
     * Should be measured from center to center.
     */
    public static final double DRIVETRAIN_WHEELBASE_METERS = Units.inchesToMeters(17.179); 

    public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 1; 
    public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 2; 
    public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 3; 
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(147.92 + 344.26 + 307.26); 

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 4; 
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 5; 
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 6; 
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(108.15 + 282.74 + 0.264 + .9); 

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 7; 
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 8; 
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 9; 
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(77.639 + 259.80 + 180.0 + 348.71 + 8.0); 

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 10; 
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 11; 
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 12; 
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(-32.881 + 172.88 + 180.0 + 4.83); 

    // *****************************************************************
    // arm constants
    public static final int HorizontalArmDriveMotorCanId = 13;
    public static final int VerticalArmDriveMotorCanId = 14;
    public static final int VirticalArmBottomMagneticSensor = 0;
    public static final int HorizontalArmMagneticSensor = 1;
    public static final int VirticalArmMiddleMagneticSensor = 2;

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

    // *******************************************************************
    // arm constants - preset positions
    public static final double armPresetPositionStowMetersHorizontalExtension = 0.0;
    public static final double armPresetPositionStowMetersVerticalExtension = 0.1952;

    public static final double armPresetPositionCubeGrabMetersHorizontalExtension = 0.3041 + Units.inchesToMeters(0.5);
    public static final double armPresetPositionCubeGrabMetersVerticalExtension = 0.0691 - Units.inchesToMeters(1.5);
    public static final double armPresetPositionCubeScoreHighMetersHorizontalExtension = 0.7262;
    public static final double armPresetPositionCubeScoreHighMetersVerticalExtension = 0.0058;
    public static final double armPresetPositionCubeScoreMediumMetersHorizontalExtension = 0.1880;
    public static final double armPresetPositionCubeScoreMediumMetersVerticalExtension = 0.0158;
    public static final double armPresetPositionCubeScoreLowMetersHorizontalExtension = 0.0979;
    public static final double armPresetPositionCubeScoreLowMetersVerticalExtension = 0.0224;

    public static final double armPresetPositionConeGrabMetersHorizontalExtension = 0.4192 - Units.inchesToMeters(0.5);
    public static final double armPresetPositionConeGrabMetersVerticalExtension = 0.0589 - Units.inchesToMeters(1.0);
    public static final double armPresetPositionConeScoreHighMetersHorizontalExtension = 0.6694;
    public static final double armPresetPositionConeScoreHighMetersVerticalExtension = 0.0;
    public static final double armPresetPositionConeScoreMediumMetersHorizontalExtension = 0.2 - Units.inchesToMeters(1);
    public static final double armPresetPositionConeScoreMediumMetersVerticalExtension = 0.01;
    public static final double armPresetPositionConeScoreLowMetersHorizontalExtension = 0.0979;
    public static final double armPresetPositionConeScoreLowMetersVerticalExtension = 0.0224;

    // *******************************************************************
    // pneumatics constants
    public static final int PneumaticsControlModuleNumber = 0;
    public static final PneumaticsModuleType PneumaticsControlModuleType = PneumaticsModuleType.REVPH;

    // *******************************************************************
    // Stabilizer constants
    public static final int StabilizerPneumaticsControlModuleForwardChannel = 0;
    public static final int StabilizerPneumaticsControlModuleReverseChannel = 1;

    // *******************************************************************
    // picker constants
    public static final int PickerHorizontalPneumaticsControlModuleForwardChannel = 2;
    public static final int PickerHorizontalPneumaticsControlModuleReverseChannel = 3;
    public static final int PickerVerticalPneumaticsControlModuleForwardChannel = 4;
    public static final int PickerVerticalPneumaticsControlModuleReverseChannel = 5; 
    
    // *****************************************************************
    // Every Bot picker constants
    public static final int EveryBotPickerMotorCanId = 15;

    // ********************************************************************
    // Controller Constants
    public static final double rumbleTimeSeconds = 0.15;

    // ********************************************************************
    // PowerDistributionPanel Constants
    public static final int currentPowerDistributionPanelCanId = 19;
    public static final ModuleType currentPowerDistributionPanelType = ModuleType.kRev;
    public static final double overcurrentRumbleTimeSeconds = 0.25;
    public static final int EveryBotMotorPdpPortId = 8;
    public static final double EveryBotMotorMaximuCurrentAmps = 45.5;

}