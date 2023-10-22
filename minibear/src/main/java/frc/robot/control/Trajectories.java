package frc.robot.control;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.common.SwerveTrajectoryConfig;
import frc.robot.common.SwerveTrajectoryGenerator;
import frc.robot.subsystems.DrivetrainSubsystem;
import java.util.ArrayList;

public class Trajectories {
    private Pose2d Node1Position;
    private Pose2d Node2Position;
    private Pose2d Node5Position;
    private Pose2d Node8Position;
    private Pose2d Node9Position;
    private Pose2d InfrontOfRampPosition;
    private Pose2d RampFarWaypoint;
    private Trajectory LeftTrajectory;
    private Trajectory Node2Trajectory;
    private Trajectory RightTrajectory;
    private Trajectory Node8Trajectory;
    private Trajectory MiddleTrajectoryPart1;
    private Trajectory MiddleTrajectoryPart2;
    private Trajectory LeftToOntoRampTrajectory;
    private Trajectory RightToOntoRampTrajectory;
    private Trajectory BehindToOntoRampTrajectory;
    private Trajectory DirectToRampTrajectory;
    private Trajectory MiddlePathBehindToOntoRampTrajectory;
    
    public SwerveTrajectoryConfig config;
    public SwerveTrajectoryConfig firstSegmentConfig;
    public SwerveTrajectoryConfig middleSegmentConfig;
    public SwerveTrajectoryConfig lastSegmentConfig;

    private DrivetrainSubsystem drivetrain;

    public Trajectories(DrivetrainSubsystem drivetrain){
        this.drivetrain = drivetrain; 

        config = drivetrain.getTrajectoryConfig();
        // trajectory config with a fast starting velocity for ramp driving. 
        // have to get a new config so that changes to this one don't affect the original
        SwerveTrajectoryConfig fastConfig = drivetrain.getTrajectoryConfig();
        fastConfig.setStartVelocity(fastConfig.getMaxVelocity() * 0.6); // less than max speed
        // trajectory config that will start limit to a slow velocity for driving off ramp
        double offOfRampSpeed = 1.25; 
        SwerveTrajectoryConfig offOfRampConfig = new SwerveTrajectoryConfig(
            offOfRampSpeed, 
            config.getMaxAcceleration(),
            config.getMaxRotationalVelocity(),
            config.getMaxRotationalAcceleration());
        // trajectory configs for joining trajectory segments together without slowing down between segments
        double trajectoryJoinSpeed = config.getMaxVelocity() * 0.4;
        firstSegmentConfig = drivetrain.getTrajectoryConfig();
        firstSegmentConfig.setEndVelocity(trajectoryJoinSpeed);
        middleSegmentConfig = drivetrain.getTrajectoryConfig();
        middleSegmentConfig.setStartVelocity(trajectoryJoinSpeed).setEndVelocity(trajectoryJoinSpeed);
        lastSegmentConfig = drivetrain.getTrajectoryConfig();
        lastSegmentConfig.setStartVelocity(trajectoryJoinSpeed);

        this.Node1Position = new Pose2d(1.678, 4.994, Rotation2d.fromDegrees(180));
        this.Node2Position = new Pose2d(1.678, 4.433, Rotation2d.fromDegrees(180));
        this.Node5Position = new Pose2d(1.678, 2.750, Rotation2d.fromDegrees(180));
        this.Node8Position = new Pose2d(1.678, 1.067, Rotation2d.fromDegrees(180));
        this.Node9Position = new Pose2d(1.678, 0.506, Rotation2d.fromDegrees(180));
        this.InfrontOfRampPosition = new Pose2d(2.0, 2.41, Rotation2d.fromDegrees(0)); 
        
        // There is slippage getting onto ramp, so we need to overshoot the center 
        // to ensure the robot gets far enough onto the ramp.   
        this.RampFarWaypoint = new Pose2d(4.122, 2.41, Rotation2d.fromDegrees(0)); 

        // behind ramp position for node 5 path
        Pose2d MiddlePathOverRampPosition = new Pose2d(6.2 - Units.inchesToMeters(6), 2.41, Rotation2d.fromDegrees(180));
        Pose2d MiddlePathRampNearWaypoint = new Pose2d(4.3 - Units.inchesToMeters(6), 2.41, Rotation2d.fromDegrees(180));
        
        Pose2d RampNearWaypoint = new Pose2d(3.34, 2.41, Rotation2d.fromDegrees(180));
        // behind ramp position for node 1,2,8,9 paths
        Pose2d BehindTrajectoryEndPosition = new Pose2d(5.27, 2.41, Rotation2d.fromDegrees(180));
        
        // Left waypoints drive from Node 1 or 2 to a location out of the community
        ArrayList<Translation2d> LeftWaypoints = new ArrayList<Translation2d>();
        LeftWaypoints.add(new Translation2d(2.1, 4.67));
        LeftWaypoints.add(new Translation2d(3.7, 4.67));
        Pose2d LeftTrajectoryEndPosition = new Pose2d(5.3, 4.67, Rotation2d.fromDegrees(0));
        this.LeftTrajectory = SwerveTrajectoryGenerator.generateTrajectory(Node1Position, LeftWaypoints, LeftTrajectoryEndPosition, config);
        this.Node2Trajectory = SwerveTrajectoryGenerator.generateTrajectory(Node2Position, LeftWaypoints, LeftTrajectoryEndPosition, config);
        
        // Right waypoints drive from Node 8 or 9 to a location out of the community
        ArrayList<Translation2d> RightWaypoints = new ArrayList<Translation2d>();
        RightWaypoints.add(new Translation2d(2.1, .69));
        RightWaypoints.add(new Translation2d(3.7, .69));
        Pose2d RightTrajectoryEndPosition = new Pose2d(5.3, .69, Rotation2d.fromDegrees(0));
        this.RightTrajectory = SwerveTrajectoryGenerator.generateTrajectory(Node9Position, RightWaypoints, RightTrajectoryEndPosition, config);
        this.Node8Trajectory = SwerveTrajectoryGenerator.generateTrajectory(Node8Position, RightWaypoints, RightTrajectoryEndPosition, config);

        // To drive from left or right onto ramp, use a common central waypoint
        ArrayList<Translation2d> BehindToRampWaypoints = new ArrayList<Translation2d>();
        BehindToRampWaypoints.add(new Translation2d(5.81, 2.748));
        this.LeftToOntoRampTrajectory = SwerveTrajectoryGenerator.generateTrajectory(LeftTrajectoryEndPosition, BehindToRampWaypoints, BehindTrajectoryEndPosition, config);
        this.RightToOntoRampTrajectory = SwerveTrajectoryGenerator.generateTrajectory(RightTrajectoryEndPosition, BehindToRampWaypoints, BehindTrajectoryEndPosition, config);
        
        // Drive onto ramp from behind
        ArrayList<Pose2d> BehindToOntoRampWaypoints = new ArrayList<Pose2d>();
        BehindToOntoRampWaypoints.add(BehindTrajectoryEndPosition);
        BehindToOntoRampWaypoints.add(RampNearWaypoint);
        // use fastConfig for this trajectory 
        this.BehindToOntoRampTrajectory = SwerveTrajectoryGenerator.generateTrajectory(BehindToOntoRampWaypoints, fastConfig); 

        this.LeftToOntoRampTrajectory = this.LeftToOntoRampTrajectory.concatenate(BehindToOntoRampTrajectory);
        this.RightToOntoRampTrajectory = this.RightToOntoRampTrajectory.concatenate(BehindToOntoRampTrajectory);

        // Drive onto ramp from in front 
        ArrayList<Pose2d> InfrontToOntoRampWaypoints = new ArrayList<Pose2d>();
        InfrontToOntoRampWaypoints.add(InfrontOfRampPosition);
        InfrontToOntoRampWaypoints.add(RampFarWaypoint);
        // use fastConfig for this trajectory 
        Trajectory InfrontToOntoRampTrajectory = SwerveTrajectoryGenerator.generateTrajectory(InfrontToOntoRampWaypoints, fastConfig);

        ArrayList<Pose2d> Node5ToFrontOfRampWaypoints = new ArrayList<Pose2d>();
        Node5ToFrontOfRampWaypoints.add(Node5Position);
        Node5ToFrontOfRampWaypoints.add(InfrontOfRampPosition);
        Trajectory Node5ToFrontOfRampTrajectory = SwerveTrajectoryGenerator.generateTrajectory(Node5ToFrontOfRampWaypoints, config);
        
        this.DirectToRampTrajectory = Node5ToFrontOfRampTrajectory.concatenate(InfrontToOntoRampTrajectory);
        
        // Construct the middle up and over ramp trajectory
        ArrayList<Pose2d> MiddleWaypoints = new ArrayList<Pose2d>();
        MiddleWaypoints.add(RampFarWaypoint);
        MiddleWaypoints.add(MiddlePathOverRampPosition);
        Trajectory RampToBehindRampTrajectory = SwerveTrajectoryGenerator.generateTrajectory(MiddleWaypoints, offOfRampConfig);
        // Drive onto ramp from behind
        ArrayList<Pose2d> MiddlePathBehindToOntoRampWaypoints = new ArrayList<Pose2d>();
        MiddlePathBehindToOntoRampWaypoints.add(MiddlePathOverRampPosition);
        MiddlePathBehindToOntoRampWaypoints.add(MiddlePathRampNearWaypoint);
        // use fastConfig for this trajectory 
        this.MiddlePathBehindToOntoRampTrajectory = SwerveTrajectoryGenerator.generateTrajectory(MiddlePathBehindToOntoRampWaypoints, fastConfig); 

        this.MiddleTrajectoryPart1 = Node5ToFrontOfRampTrajectory
            .concatenate(InfrontToOntoRampTrajectory);
        this.MiddleTrajectoryPart2 = RampToBehindRampTrajectory;
    }

    public SwerveTrajectoryConfig getConfig() {
        return config;
    }
    
    public Trajectory getBehindToOntoRampTrajectory() {
        return BehindToOntoRampTrajectory;
    }

    public Trajectory getDirectToRampTrajectory() {
        return DirectToRampTrajectory;
    }

    public SwerveTrajectoryConfig getFirstSegmentConfig() {
        return firstSegmentConfig;
    }

    public Pose2d getInfrontOfRampPosition() {
        return InfrontOfRampPosition;
    }

    public SwerveTrajectoryConfig getLastSegmentConfig() {
        return lastSegmentConfig;
    }

    public Trajectory getLeftToOntoRampTrajectory() {
        return LeftToOntoRampTrajectory;
    }

    public Trajectory getLeftTrajectory() {
        return LeftTrajectory;
    }

    public Trajectory getMiddlePathBehindToOntoRampTrajectory() {
        return MiddlePathBehindToOntoRampTrajectory;
    }

    public SwerveTrajectoryConfig getMiddleSegmentConfig() {
     
        return middleSegmentConfig;
    }
    
    public Trajectory getMiddleTrajectoryPart1() {
        return MiddleTrajectoryPart1;
    }

    public Trajectory getMiddleTrajectoryPart2() {
        return MiddleTrajectoryPart2;
    }

    public Pose2d getNode1Position() {
        return Node1Position;
    }

    public Pose2d getNode2Position() {
        return Node2Position;
    }

    public Trajectory getNode2Trajectory() {
        return Node2Trajectory;
    }

    public Pose2d getNode5Position() {
        return Node5Position;
    }

    public Pose2d getNode8Position() {
        return Node8Position;
    }

    public Trajectory getNode8Trajectory() {
        return Node8Trajectory;
    }

    public Pose2d getNode9Position() {
        return Node9Position;
    }

    public Pose2d getRampFarWaypoint() {
        return RampFarWaypoint;
    }
    
    public Trajectory getRightToOntoRampTrajectory() {
        return RightToOntoRampTrajectory;
    }

    public Trajectory getRightTrajectory() {
        return RightTrajectory;
    }
}   
