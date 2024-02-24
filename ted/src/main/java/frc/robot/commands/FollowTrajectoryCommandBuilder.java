// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: FollowTrajectoryCommandBuilder.java
// Intent: A builder that returns a command to follow a PathPlanner trajectory
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import com.pathplanner.lib.commands.FollowPathHolonomic;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * Forms a class to buid trajectory-following commands
 */
public class FollowTrajectoryCommandBuilder {
    public static HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(                        
                            new PIDConstants(2.0, 0, 0),  // Translation PID constants
                            new PIDConstants(4.5, 0.001, 0), // Rotation PID constants
                            2.3, // Max module speed, in m/s
                            0.43, // Drive base radius in meters. Distance from robot center to furthest module.
                            new ReplanningConfig() // Default path replanning config. See the API for the options here
                        );

    /**
     * A method to build a follow trajectory command
     * 
     * @param traj - path planner trajectory
     * @param drivetrain - drivetrain subsystem
     * @param isFirstPath - true if this is the first path in the sequence (resets the odometry to traj starting point)
     * @return - a command to follow the trajectory
     */
    public static CommandBase build(PathPlannerPath traj, DrivetrainSubsystem drivetrain, boolean isFirstPath) {
        return new SequentialCommandGroup(
                new InstantCommand(() -> {
                    // Reset odometry for the first path you run during auto
                    if (isFirstPath) {
                        drivetrain.setRobotPosition(traj.getPreviewStartingHolonomicPose());
                    }
                }),
                new FollowPathHolonomic(
                        traj,
                        drivetrain::getRobotPosition, // Pose supplier
                        drivetrain::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                        drivetrain::drive, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds
                        pathFollowerConfig,
                        () -> mirrorPathForRedAliance(),
                        (Subsystem) drivetrain
                ));
    }

    /**
     * A method to build a follow trajectory command
     * @param traj - path planner trajectory
     * @param drivetrain - drivetrain subsystem
     * @return - a command to follow the trajectory
     */
    public static CommandBase build(PathPlannerPath traj, DrivetrainSubsystem drivetrain) {
        return build(traj, drivetrain, false);
    }

    /**
     * A method that returns true when we are on the red alliance
     * Used for paths that should be mirrored when we are on red alliance
     */
    public static boolean mirrorPathForRedAliance(){        
        var alliance = DriverStation.getAlliance();
        if (alliance != Alliance.Invalid) {
            return alliance == Alliance.Red;
        }
        return false;
    }

    /**
     * A method that always returns false
     * Used for paths that should never be mirrored
     */
    public static boolean neverMirrorPath(){        
        return false;
    }
}