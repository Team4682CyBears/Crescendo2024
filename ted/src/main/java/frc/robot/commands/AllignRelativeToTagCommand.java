// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up 2023
// File: AllignRelativeToTag.java
// Intent: Forms a command to use vision for position updates
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.subsystems.CameraSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.common.MotorUtils;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
/**
 * Implements a command that will allign the robot relativly to a given tag, offset with a given pose
 */
public class AllignRelativeToTagCommand extends Command{
  private boolean done = false;
  private double wantedTagId;
  private Pose2d targetPoseInVisionSpace;
  private Pose2d targetPositionInRobotSpace;
  private Pose2d startingPositionInRobotSpace;
  private final double velocityFactor = 0.3;
  private PIDController xPID = new PIDController(1.0, 0.0, 0.0);
  private PIDController yPID = new PIDController(1.0, 0.0, 0.0);
  private PIDController rotationPID = new PIDController(1.0, 0.0, 0.0);
  private final double transformTolerance = 0.01;
  private final double rotationTolerance = 5;

  private DrivetrainSubsystem drivetrainsubsystem = null;
  private CameraSubsystem camerasubsystem = null;;

  /**
   * Constructor for command that alligns relative to a given tag with given offsets
   * @param drivetrainSubsystem the drivetrain subsystem
   * @param camerasubsystem the camera subystem
   * @param targetPoseInVisionSpace the pose that you want the robot to be relative to the tag
   * @param tagId which tag you want to be relative to
   */
  public AllignRelativeToTagCommand(DrivetrainSubsystem drivetrainSubsystem, CameraSubsystem camerasubsystem, Pose2d targetPoseInVisionSpace, double tagId) {
    this.drivetrainsubsystem = drivetrainSubsystem;
    this.camerasubsystem = camerasubsystem;
    this.wantedTagId = tagId;
    this.targetPoseInVisionSpace = targetPoseInVisionSpace;

    rotationPID.enableContinuousInput(-180, 180);

    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    done = false;
    Pose2d robotPoseInVisionSpace = camerasubsystem.getVisionBotPoseInTargetSpace();
    if (robotPoseInVisionSpace != null && camerasubsystem.getTagId() == wantedTagId){
      // find the difference between current pose in vision space and target pose in vision space
      Twist2d desiredTwist = robotPoseInVisionSpace.log(targetPoseInVisionSpace);
      startingPositionInRobotSpace = drivetrainsubsystem.getRobotPosition();
      // apply the twist to the robot's current position
      targetPositionInRobotSpace = startingPositionInRobotSpace.exp(desiredTwist);
    }
    else {
      System.out.println("no tag in sight");
      done = true;
    }

    drivetrainsubsystem.drive(new ChassisSpeeds(0, 0, 0));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
      startingPositionInRobotSpace = drivetrainsubsystem.getRobotPosition();
      double xVelocity = 0.0;
      double yVelocity = 0.0;
      double rotVelocity = 0.0;

      if(Math.abs(targetPositionInRobotSpace.getX() - startingPositionInRobotSpace.getX()) >= transformTolerance){
        xVelocity = xPID.calculate(startingPositionInRobotSpace.getX(), targetPositionInRobotSpace.getX());
        xVelocity = -1 * MotorUtils.clamp(xVelocity, -velocityFactor, velocityFactor);
      }

      if(Math.abs(targetPositionInRobotSpace.getY() - startingPositionInRobotSpace.getY()) >= transformTolerance){
        yVelocity = yPID.calculate(startingPositionInRobotSpace.getY(), targetPositionInRobotSpace.getY());
        yVelocity = -1 * MotorUtils.clamp(yVelocity, -velocityFactor, velocityFactor);
      }

      if(Math.abs(targetPositionInRobotSpace.getRotation().getDegrees() - startingPositionInRobotSpace.getRotation().getDegrees()) >= rotationTolerance){
        rotVelocity = rotationPID.calculate(startingPositionInRobotSpace.getRotation().getDegrees(), targetPositionInRobotSpace.getRotation().getDegrees());
        rotVelocity = -1 * MotorUtils.clamp(rotVelocity, -velocityFactor, velocityFactor);
      }

      drivetrainsubsystem.drive(new ChassisSpeeds(xVelocity, yVelocity, rotVelocity));
    }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){
    done = true;
    drivetrainsubsystem.drive(new ChassisSpeeds(0, 0, 0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished(){
    startingPositionInRobotSpace = drivetrainsubsystem.getRobotPosition();
    done = Units.radiansToDegrees(Math.abs(MathUtil.angleModulus((targetPositionInRobotSpace.getRotation().getRadians() - startingPositionInRobotSpace.getRotation().getDegrees())))) <= rotationTolerance
      && Math.abs(targetPositionInRobotSpace.getX() - startingPositionInRobotSpace.getX()) <= transformTolerance
      && Math.abs(targetPositionInRobotSpace.getY() - startingPositionInRobotSpace.getY()) <= transformTolerance;
    return done;
  }
}