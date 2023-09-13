// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rapid React - 2023
// File: DriveTrajectoryCommand.java
// Intent: Forms a command to drive the wheels according to a constructed trajectory.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;
import static java.lang.Math.abs;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.common.SwerveTrajectoryConfig;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveTrajectoryCommand extends CommandBase{
  private DrivetrainSubsystem drivetrain;
  private Trajectory movementPlan;
  private Timer timer = new Timer();
  private boolean done = false;
  private double expectedDuration = 0.0;

  private PIDController xPidController = new PIDController(2.0,0.0,0.0);
  private PIDController yPidController = new PIDController(2.0,0.0,0.0);
  private ProfiledPIDController thetaPidController;
  private HolonomicDriveController controller;

  private Pose2d finalPosition = null;
  private Pose2d overTimeDelta = Constants.TrajectoryPoseTol;

  /** 
  * Creates a new driveCommand. 
  * 
  * @param drivetrainSubsystem - the drive train subsystem
  * @param plan - the Trajectory that should be followed by the robot
  */
  public DriveTrajectoryCommand(
    DrivetrainSubsystem drivetrainSubsystem,
    Trajectory plan) {
    this.drivetrain = drivetrainSubsystem;
    this.movementPlan = plan;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrainSubsystem);
    
    // setup theta PID controller and holonomic controller
    SwerveTrajectoryConfig config = drivetrain.getTrajectoryConfig();
    Constraints movementConstraints = new TrapezoidProfile.Constraints(
    config.getMaxRotationalVelocity(),
    config.getMaxRotationalAcceleration());
    thetaPidController = new ProfiledPIDController(4.5, 0.001, 0.0, movementConstraints);
    //TODO looks like HolonomicDriveController enablesContinuousInput.  Try removing this and re-testing.  
    thetaPidController.enableContinuousInput(-Math.PI, Math.PI);
    controller = new HolonomicDriveController(xPidController, yPidController, thetaPidController);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    expectedDuration = movementPlan.getTotalTimeSeconds();
    this.finalPosition = movementPlan.sample(expectedDuration).poseMeters;
    timer.reset();
    timer.start();
    done = false;

    // TODO remove this code once setting robot position is debugged
    //Check robot position vs. trajecgtory starting position and abort if they are not close:
    Pose2d currentLocation = drivetrain.getRobotPosition();
    Pose2d targetPose = movementPlan.sample(0.0).poseMeters;
    Translation2d deltaLocation = currentLocation.getTranslation().minus(targetPose.getTranslation());
    if (abs(deltaLocation.getNorm())>0.5){
      System.out.println("ERROR: ABORTING TRAJECTORY: Current position " + currentLocation + " is too far from trajectory starting position " + targetPose);
      done = true;
    }
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer.get() > expectedDuration && this.isDeltaReasonable()) {
        drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        timer.stop();
        done = true;
    }
    else {
        double currentElapsedTimeInSeconds = timer.get();
        Pose2d currentLocation = drivetrain.getRobotPosition();
        Trajectory.State targetState = movementPlan.sample(currentElapsedTimeInSeconds);
        
        // For swerve drive, call the controller like this:
        // https://github.com/wpilibsuite/allwpilib/blob/main/wpilibNewCommands/src/main/java/edu/wpi/first/wpilibj2/command/SwerveControllerCommand.java#L222
        // The key is to override each rotation with the trajectory's final rotation
        ChassisSpeeds calculatedSpeed = controller.calculate(currentLocation, targetState, finalPosition.getRotation());
        // Note: we previously used drivetrain.clampChassisSpeeds here, but that caused erratic behavior   
        drivetrain.drive(calculatedSpeed);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    timer.stop();
    if(interrupted) {
      done = true;      
    }
    System.out.println("Movement Complete: expected duration (seconds) == " + this.expectedDuration + " actual duration (seconds) == " + timer.get());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }

  private boolean isDeltaReasonable() {
    Pose2d currentPosition = this.drivetrain.getRobotPosition();
    Transform2d delta = new Transform2d(currentPosition, this.finalPosition);
    if( abs(delta.getX()) <= this.overTimeDelta.getX() &&
        abs(delta.getY()) <= this.overTimeDelta.getY() &&
        abs(MathUtil.angleModulus(delta.getRotation().getRadians())) <= MathUtil.angleModulus(this.overTimeDelta.getRotation().getRadians())) {
      return true;
    }
    return false;
  }
}
