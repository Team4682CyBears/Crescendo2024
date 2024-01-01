// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rapid React - 2023
// File: DriveToPointCommand.java
// Intent: Forms a command to drive the robot to a point
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveToPointCommand extends CommandBase
{
  private DrivetrainSubsystem drivetrain = null;
  private Timer timer = new Timer();
  private boolean done = false;
  private Pose2d startPosition = null;
  private Pose2d currentPosition = null;
  private Pose2d destinationPosition = null;
  private double totalDistanceMeters = 0.0;

  private double positionToleranceMeters = 0.05;
  private double rotationToleranceRadians = 4.0 * 0.01745329;

  private double accelerationMaximumDurationSeconds = 2.0;
  private double decelerationMaximumDurationSeconds = 2.0;
  private double accelerationThresholdDistanceMeters = 0.0;
  private double decelerationThresholdDistanceMeters = 0.0;
  private double targetMaximumVelocityMetersPerSecond = DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
  private double accelerationLinearRate = targetMaximumVelocityMetersPerSecond / accelerationMaximumDurationSeconds;
  private double decelerationLinearRate = targetMaximumVelocityMetersPerSecond / decelerationMaximumDurationSeconds;
  private double targetOperationDurationSeconds = 0.0;

  private double remainingRotationRadians = 0.0;
  private double accelerationThresholdRotationRadians = 0.0;
  private double decelerationThresholdRotationRadians = 0.0;
  private double accelerationRotationalRate = DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / accelerationMaximumDurationSeconds;
  private double decelerationRotationalRate = DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND / decelerationMaximumDurationSeconds;
  private double targetMaximumRotationRadiansPerSecond = 0.0;
  
  private boolean isTrapazoidalProfile = false;  

  private static final int CommandSchedulerPeriodMilliseconds = 20;
  private static final int CommandSchedulerCyclesPerSecond = 1000/CommandSchedulerPeriodMilliseconds;
  private static final int RecentRepresentativeSampleCount = 10; // last 10 samples ... 200 ms

  /** 
  * Creates a new driveCommand. 
  * 
  * @param drivetrainSubsystem - the drive train subsystem
  * @param targetPosition - the target destination position
  */
  public DriveToPointCommand(
    DrivetrainSubsystem drivetrainSubsystem,
    Pose2d targetDestination) {
    this.drivetrain = drivetrainSubsystem;
    this.destinationPosition = targetDestination;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // init the positions
    this.startPosition = drivetrain.getRobotPosition();
    this.currentPosition = this.startPosition;

    // calculate distance thresholds
    totalDistanceMeters  = this.getRemainingDistanceMeters();

    double trapazoidalDistanceStartThresholdMeters = 
        0.5 * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * this.accelerationMaximumDurationSeconds;
    double trapazoidalDistanceEndThresholdMeters = 
        0.5 * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND * this.decelerationMaximumDurationSeconds;
    double trapazoidalDistanceThresholdMeters = trapazoidalDistanceStartThresholdMeters + trapazoidalDistanceEndThresholdMeters;

    // establish if profile is triangular or trapazoidal
    isTrapazoidalProfile = (totalDistanceMeters > trapazoidalDistanceThresholdMeters);

    if(isTrapazoidalProfile) {
        this.accelerationThresholdDistanceMeters = totalDistanceMeters - trapazoidalDistanceStartThresholdMeters;
        this.decelerationThresholdDistanceMeters = trapazoidalDistanceEndThresholdMeters;
        this.targetMaximumVelocityMetersPerSecond = DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND;
        this.accelerationLinearRate = targetMaximumVelocityMetersPerSecond / accelerationMaximumDurationSeconds;
        this.decelerationLinearRate = targetMaximumVelocityMetersPerSecond / decelerationMaximumDurationSeconds;
        // get target duration of operation
        this.targetOperationDurationSeconds = 
            this.accelerationMaximumDurationSeconds + 
            this.decelerationMaximumDurationSeconds + 
            (totalDistanceMeters - trapazoidalDistanceStartThresholdMeters - trapazoidalDistanceEndThresholdMeters) / this.targetMaximumVelocityMetersPerSecond;
        System.out.println("Trapazoidal");
    }
    else {
        this.targetMaximumVelocityMetersPerSecond = (totalDistanceMeters / (0.5*this.accelerationMaximumDurationSeconds + 0.5*this.decelerationMaximumDurationSeconds));
        this.accelerationThresholdDistanceMeters = 0.5 * targetMaximumVelocityMetersPerSecond * this.accelerationMaximumDurationSeconds;
        this.decelerationThresholdDistanceMeters = accelerationThresholdDistanceMeters;
        this.accelerationLinearRate = targetMaximumVelocityMetersPerSecond / accelerationMaximumDurationSeconds;
        this.decelerationLinearRate = targetMaximumVelocityMetersPerSecond / decelerationMaximumDurationSeconds;
        // get target duration of operation
        this.targetOperationDurationSeconds = this.accelerationMaximumDurationSeconds + this.decelerationMaximumDurationSeconds;
        System.out.println("Triangular");
    }


    // get the initial spin necessary
    this.remainingRotationRadians = this.getRemainingRotationRadians();

    // spin acceleration rates - we will always do this triangular for simplicity
    targetMaximumRotationRadiansPerSecond = Math.abs(this.remainingRotationRadians / (0.5*this.accelerationMaximumDurationSeconds + 0.5*this.decelerationMaximumDurationSeconds));
    this.accelerationThresholdRotationRadians = 0.5 * targetMaximumRotationRadiansPerSecond * this.accelerationMaximumDurationSeconds;
    this.decelerationThresholdRotationRadians = accelerationThresholdRotationRadians;
    this.accelerationRotationalRate = targetMaximumRotationRadiansPerSecond / accelerationMaximumDurationSeconds;
    this.decelerationRotationalRate = targetMaximumRotationRadiansPerSecond / decelerationMaximumDurationSeconds;

    // make this class start executing
    done = false;
    timer.reset();
    timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // get current position
    this.currentPosition = drivetrain.getRobotPosition();
    // get recent velocity
    double recentVelocity = drivetrain.getRecentAverageVelocityInMetersPerSecond(CommandSchedulerPeriodMilliseconds * RecentRepresentativeSampleCount); 
    // get current distance
    double remainingDistance = this.getRemainingDistanceMeters();

    // establish the resultant velocity
    double targetResultantVelocityVector = this.targetMaximumVelocityMetersPerSecond;
    if(Math.abs(remainingDistance) <= this.positionToleranceMeters) {
        targetResultantVelocityVector = 0.0;
    }
    else if(remainingDistance > accelerationThresholdDistanceMeters) {
        targetResultantVelocityVector = 
            Math.max(recentVelocity + this.accelerationLinearRate, DrivetrainSubsystem.MIN_VELOCITY_BOUNDARY_METERS_PER_SECOND);
    }
    else if(remainingDistance < decelerationThresholdDistanceMeters) {
        targetResultantVelocityVector =
            Math.max(recentVelocity - this.decelerationLinearRate, DrivetrainSubsystem.MIN_VELOCITY_BOUNDARY_METERS_PER_SECOND);
    }
    else{
        targetResultantVelocityVector = this.targetMaximumVelocityMetersPerSecond;
    }

    // get recent angular velocity
    double recentAngularVelocity = drivetrain.getRecentAverageAngularVelocityInRadiansPerSecond(CommandSchedulerPeriodMilliseconds * RecentRepresentativeSampleCount);

    // establish the next spin
    this.remainingRotationRadians = this.getRemainingRotationRadians();
    double spinMultiplier = (this.remainingRotationRadians >= 0.0) ? -1.0 : 1.0;
    double targetSpinRadiansPerSecond = this.targetMaximumRotationRadiansPerSecond;
    if(Math.abs(remainingRotationRadians) <= this.rotationToleranceRadians) {
        targetSpinRadiansPerSecond = 0.0;
    }
    else if(this.remainingRotationRadians > this.accelerationThresholdRotationRadians ) {
        targetSpinRadiansPerSecond =
            Math.max(recentAngularVelocity + this.accelerationRotationalRate, DrivetrainSubsystem.MIN_ANGULAR_VELOCITY_BOUNDARY_RADIANS_PER_SECOND) *
            spinMultiplier;
    }
    else if(this.remainingRotationRadians < this.decelerationThresholdRotationRadians ) {
        targetSpinRadiansPerSecond =
            Math.max(recentAngularVelocity - this.decelerationRotationalRate, DrivetrainSubsystem.MIN_ANGULAR_VELOCITY_BOUNDARY_RADIANS_PER_SECOND) *
            spinMultiplier;
    }
    else {
        targetSpinRadiansPerSecond = this.targetMaximumRotationRadiansPerSecond * spinMultiplier;
    }

    if(!done && (targetResultantVelocityVector != 0.0 || targetSpinRadiansPerSecond != 0.0)) {
        // get transform for current state and final state
        double xVelocity = 0.0;
        double yVelocity = 0.0;
        if(targetResultantVelocityVector != 0.0) {
            // get transform for current state and final state so we can scale the x and y velocity
            Transform2d currentTransform = new Transform2d(this.currentPosition, this.destinationPosition);
            xVelocity = targetResultantVelocityVector * (currentTransform.getX()/remainingDistance);
            yVelocity = targetResultantVelocityVector * (currentTransform.getY()/remainingDistance);      
        }
        drivetrain.drive(new ChassisSpeeds(xVelocity, yVelocity, targetSpinRadiansPerSecond));
    }
    else {
        drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        done = true;
        timer.stop();
        System.out.println("********************* DONE!! *************************");
    }   
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted) {
        drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        done = true;      
        timer.stop();
    }
    System.out.println("Target operation duration (seconds) = " + this.targetOperationDurationSeconds + " Actual operation duration (seconds) = " + timer.get());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }

  private double getRemainingDistanceMeters() {
    return currentPosition.getTranslation().getDistance(destinationPosition.getTranslation());
  }

  private double getRemainingRotationRadians() {
    return currentPosition.getRotation().getRadians() - destinationPosition.getRotation().getRadians();
  }

}
