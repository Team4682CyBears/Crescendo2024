// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: ManualInputInterfaces.java
// Intent: Forms a class that generates trajectories for swerve drives.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.common;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.Trajectory.State;

/**
 * A collection of methods to generate trajectories for swerve drives.  
 * The wpilib TrajectoryGenerator is designed for non-holonomic drivetrains
 * and generates extraneous "fishtail" movements that are unnecessary for swerve drives.  In this class the translation motions 
 * of the trajectory are uncoupled from the initial and final robot pose to get trajectories that are appropriate for swerve.
 * The trajectory time is extended to allow the final rotation to complete.
 * To drive these trajectories, call the HolonomicDriveController like this:
 * https://github.com/wpilibsuite/allwpilib/blob/main/wpilibNewCommands/src/main/java/edu/wpi/first/wpilibj2/command/SwerveControllerCommand.java#L222
 * controller.calculate(currentLocation, targetStateFromTrajectorySample, finalPosition.getRotation());
 * The key is to override each rotation with the trajectory's final rotation
 * Note: we previously wrapped this call in drivetrain.clampChassisSpeeds(), but that caused erratic behavior  
 * Test changes to this class using trajectories in TestTrajectories.java
 */
public class SwerveTrajectoryGenerator {
    // small translation to add to states to avoid adjacent states having same x/y
    private static Transform2d epsilonTranslation = new Transform2d(new Translation2d(0.0, 0.01), new Rotation2d(0.0));
    private static double zeroAcceleration = 0.0;
    private static double zeroVelocity = 0.0;
    private static double zeroCurvature = 0.0;

    /**
     * Generates trajectories for swerve drives.     
     * @param start
     * @param interiorWaypoints
     * @param end
     * @param config
     * @return trajectory 
     */
    public static Trajectory generateTrajectory(Pose2d start, List<Translation2d> interiorWaypoints, Pose2d end, SwerveTrajectoryConfig config){
        Rotation2d origStartingAngle = start.getRotation();
        Rotation2d origEndingAngle = end.getRotation();
        // calaculte new headings for the start and end points 
        // this tricks the trajectory generator into not generating extraneous motions that are unnecessary for swerve drives 
        Rotation2d newStartingAngle = interiorWaypoints.get(0).minus(start.getTranslation()).getAngle();
        Rotation2d newEndingAngle = end.getTranslation().minus(interiorWaypoints.get(interiorWaypoints.size()-1)).getAngle();

        start = new Pose2d(start.getTranslation(), newStartingAngle);
        end = new Pose2d(end.getTranslation(), newEndingAngle);
        Trajectory t = TrajectoryGenerator.generateTrajectory(start, interiorWaypoints, end, config); 
        List<Trajectory.State> states = t.getStates();
        states = overrideFinalStateRotation(states, origEndingAngle);
        //Calaculate time for rotational trapezoidal profile. 
        double rotationTime = CalculateRotationTime(config, origStartingAngle, origEndingAngle);
        if (rotationTime > t.getTotalTimeSeconds()){
            states = AddFinalRotationalState(states, rotationTime);
        }
        return new Trajectory(states);
    }

    /**
    * Generates trajectories for swerve drives.  Can handle the special case with only two waypoints, 
    * that have the same x/y (rotational only).  
     * @param waypoints
     * @param config
     * @return trajectory
     */
    public static Trajectory generateTrajectory(ArrayList<Pose2d> waypoints, SwerveTrajectoryConfig config){
        int len = waypoints.size();
        Rotation2d origStartingAngle = waypoints.get(0).getRotation();
        Rotation2d origEndingAngle = waypoints.get(len-1).getRotation();
        // calaculte new headings for the start and end points 
        // this tricks the trajectory generator into not generating extraneous motions that are unnecessary for swerve drives 
        Rotation2d newStartingAngle = waypoints.get(1).getTranslation().minus(waypoints.get(0).getTranslation()).getAngle();
        Rotation2d newEndingAngle = waypoints.get(len-1).getTranslation().minus(waypoints.get(len-2).getTranslation()).getAngle();
        //Calaculate time for rotational trapezoidal profile. 
        double rotationTime = CalculateRotationTime(config, origStartingAngle, origEndingAngle);

        if (len == 2 && hasSameXY(waypoints.get(0), waypoints.get(1))){
            // handle special case where there are only two waypoints, and they have the same x/y (rotational only)
            // don't call trajectory generator and create the two trajectory states manually 
            Pose2d startPose = waypoints.get(1); // want startPose to have ending angle
            Pose2d endPose = waypoints.get(1).plus(epsilonTranslation);
            ArrayList<Trajectory.State> states = new ArrayList<Trajectory.State>();
            states.add(new Trajectory.State(0.0, zeroVelocity, zeroAcceleration, startPose, zeroCurvature));
            states.add(new Trajectory.State(rotationTime, zeroVelocity, zeroAcceleration, endPose, zeroCurvature));
            return new Trajectory(states);
        } 
        else {
            waypoints.set(0, new Pose2d(waypoints.get(0).getTranslation(), newStartingAngle));
            waypoints.set(len-1, new Pose2d(waypoints.get(len-1).getTranslation(), newEndingAngle));
         
            Trajectory t = TrajectoryGenerator.generateTrajectory(waypoints, config); 
            List<Trajectory.State> states = t.getStates();
            states = overrideFinalStateRotation(states, origEndingAngle);
            if (rotationTime > t.getTotalTimeSeconds()){
                states = AddFinalRotationalState(states, rotationTime);
            }
            return new Trajectory(states);
        }
    }   

    /**
     * prints a trajectory one state per line
     * @param t Trajectory
     */
    public static void printTrajectory(Trajectory t){
        for (int i = 0; i < t.getStates().size(); i++) {
            System.out.println(t.getStates().get(i));
        }
    }

    /**
     * Samples and prints the trajectory at evenly-speaced times
     * It's recommended to print sample trajectories to validate newly-created trajectories.
     * trajectory.sample has some bugs that result in some trajectory samples having NaN for x/y.
     * @param trajectory
     * @param numSamples - number of samples to take
     */
    public static void printSampledTrajectory(Trajectory trajectory, int numSamples){
        double smallScaleUpFactor = 1.0001;
        for (double t = 0; t <= trajectory.getTotalTimeSeconds() * smallScaleUpFactor; t += trajectory.getTotalTimeSeconds()/(numSamples-1)){
            System.out.println(trajectory.sample(t));
        }
    }

    /**
     * Adds a final rotational state to the list of Trajectory.States to allow additional time for final rotation to complete.
     * @param states
     * @param rotationTime
     * @return modified states 
     */
    private static List<Trajectory.State> AddFinalRotationalState(List<Trajectory.State> states, double rotationTime){
        // Add a final extended state with time to complete the full rotation. 
        State finalState = states.get(states.size()-1);
        // both states need to have zero acceleration so that trajectory.sample will not return NaN
        State extendedState = new State(rotationTime, finalState.velocityMetersPerSecond, 
        zeroAcceleration, finalState.poseMeters, finalState.curvatureRadPerMeter);
        // move finalState by a tiny translation so that trajectory.sample will not return NaN  
        finalState.poseMeters = finalState.poseMeters.plus(epsilonTranslation);
        finalState.accelerationMetersPerSecondSq = zeroAcceleration; 
        states.set(states.size()-1, finalState);
        states.add(extendedState);
        return states;    
    }

    /**
     * Overrides the final state with a new rotation.  
     * @param states
     * @param finalRotation
     * @return modified states
     */
    private static List<Trajectory.State> overrideFinalStateRotation(List<Trajectory.State> states, Rotation2d finalRotation){
        State finalState = states.get(states.size()-1);
        finalState.poseMeters = new Pose2d(finalState.poseMeters.getTranslation(), finalRotation);
        states.set(states.size()-1, finalState);
        return states; 
    }

    /**
     * Generates a trapezoid profile for rotation movement
     * @param config
     * @param startAngle
     * @param endAngle
     * @return trapezoid profile
     */
    public static TrapezoidProfile GenerateRotationTrapezoidProfile(SwerveTrajectoryConfig config, Rotation2d startAngle, Rotation2d endAngle){
        TrapezoidProfile.Constraints thetaProfileConstraints = new TrapezoidProfile.
        Constraints(config.getMaxRotationalVelocity(),config.getMaxRotationalAcceleration());
        TrapezoidProfile thetaProfile = new TrapezoidProfile(
            thetaProfileConstraints, 
            new TrapezoidProfile.State(endAngle.getRadians(), 0.0),
            new TrapezoidProfile.State(startAngle.getRadians(), 0.0));
        return thetaProfile;
    }

    /**
     * //Calaculate time for rotational trapezoidal profile. 
     * @param config
     * @param startAngle
     * @param endAngle
     * @return the total rotational time
     */
    public static double CalculateRotationTime(SwerveTrajectoryConfig config, Rotation2d startAngle, Rotation2d endAngle){
        TrapezoidProfile thetaProfile = GenerateRotationTrapezoidProfile(config, startAngle, endAngle);
        return thetaProfile.totalTime();
    }

    /**
     * Returns true if both poses have the same x/y coordinates
     * @param pose1
     * @param pose2
     * @return true if both the coordinates are the same
     */
    private static boolean hasSameXY(Pose2d pose1, Pose2d pose2){
        return (pose1.getX() == pose2.getX()) && (pose1.getY() == pose2.getY());
    }
}
