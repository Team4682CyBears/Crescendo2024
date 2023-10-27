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
    public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(335.21);

    public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 4;
    public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 5;
    public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 6;
    public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(29.09+180);

    public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 7;
    public static final int BACK_LEFT_MODULE_STEER_MOTOR = 8;
    public static final int BACK_LEFT_MODULE_STEER_ENCODER = 9;
    public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(236.25+5.09+180);

    public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 10;
    public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 11;
    public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 12;
    public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(322.03);

    // *****************************************************************
    // standard stuff constants - motors rotation, etc.
    public static final double DegreesPerRevolution = 360.0;
    // NEO maximum RPM
    public static final double neoMaximumRevolutionsPerMinute = 5676;
    // NEO 550 maximum RPM - see:
    // https://www.revrobotics.com/rev-21-1651/#:~:text=The%20following%20specifications%20for%20the%20NEO%20550%20Brushless,Motor%20Kv%3A%20917%20Kv%20Free%20Speed%3A%2011000%20RPM
    public static final double neoFiveFiveZeroMaximumRevolutionsPerMinute = 11000;
    // this uses the halls effect sensor when plugged into the spark max
    // see: https://www.revrobotics.com/rev-21-1650/ where it says "42 counts per
    // rev."
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

    // *****************************************
    // Fine placement constants
    // Center of rotation for fine placement
    public static final Translation2d RobotFrontRotationalCenter = new Translation2d(
            DRIVETRAIN_WHEELBASE_METERS / 2 + snoutDepth + Units.inchesToMeters(10), 0.0);
    // velocity for fine placement
    public static final double FinePlacementRotationalVelocity = 0.7;

    // *******************************************************************
    // Stabilizer constants
    public static final int StabilizerPneumaticsControlModuleForwardChannel = 0;
    public static final int StabilizerPneumaticsControlModuleReverseChannel = 1;

    // *****************************************************************
    // Picker Constants
    public static final int SrxMotor1CanId = 15;
    public static final int SrxMotor2CanId = 16;
    public static final double CURRENT_SPIKE_THRESHOLD = 60.0; // TODO Example value, adjust as needed

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

    // ********************************************************************
    //Wrist Constants
    public static int wristMotorCanID = 19;


    // THESE ARE NOT DEGRESS THEY ARE JUST ARBITRARY UNITS 
    public static final double WRIST_ANGLE_PICKUP = 160.0; // Example angle for picking up items
    public static final double WRIST_ANGLE_1 = 80.0; // Example angle for storing position //TODO might need to be negitive if zero is at the top positon?
    public static final double WRIST_ANGLE_2 = 80.0; // Example angle for deploying items
    public static final double WRIST_ANGLE_3 = 0.0; // Example angle for deploying items

    public static final double INTAKE_SPEED = 0.3; //should be positive

    public static final double SHOOT_SPEED_0 = 0.3;
    public static final double SHOOT_SPEED_1 = 1.0;
    public static final double SHOOT_SPEED_2 = 0.6;
    public static final double SHOOT_SPEED_3 = 1.0;


    

    

}