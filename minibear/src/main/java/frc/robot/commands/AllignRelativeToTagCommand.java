// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up 2023
// File: AllignRelativeToTag.java
// Intent: Forms a command to use vision for position updates
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.common.VisionMeasurement;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.common.MotorUtils;

/**
 * Implements a command.
 */
public class AllignRelativeToTagCommand extends CommandBase{
  private boolean done = false;
  private double tagID;
  private double targetX;
  private double targetY;
  private double targetRot;
  private final double velocityFactor = 0.3;
  private PIDController transformPID = new PIDController(1.0, 0.0, 0.0);
  private PIDController rotationPID = new PIDController(1.0, 0.0, 0.0);
  private final double tolerance = 0.05;

  private DrivetrainSubsystem drivetrainsubsystem = null;
  private CameraSubsystem camerasubsystem = null;;

  /**
   * Constructor for command.
   */
  public AllignRelativeToTagCommand(DrivetrainSubsystem drivetrainSubsystem, CameraSubsystem camerasubsystem, double targetX, double targetY, double targetRot, double tagID) {
    this.drivetrainsubsystem = drivetrainSubsystem;
    this.camerasubsystem = camerasubsystem;
    this.tagID = tagID;
    this.targetX = targetX;
    this.targetY = targetY;
    this.targetRot = targetRot;

    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Made allign command");
    done = false;
    drivetrainsubsystem.drive(new ChassisSpeeds(0, 0, 0));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){
    VisionMeasurement relativeMeasurement = camerasubsystem.getVisionBotPoseInTargetSpace();
    if (relativeMeasurement.getRobotPosition() != null){
      double xVelocity = 0.0;
      double yVelocity = 0.0;
      double rotVelocity = 0.0;

      if(Math.abs(targetX - relativeMeasurement.getRobotPosition().getX()) >= tolerance){
        xVelocity = transformPID.calculate(relativeMeasurement.getRobotPosition().getX(), 0.0);
        xVelocity = -1 * MotorUtils.clamp(xVelocity, -velocityFactor, velocityFactor);
      }

      if(Math.abs(targetY - relativeMeasurement.getRobotPosition().getY()) >= tolerance){
        yVelocity = transformPID.calculate(relativeMeasurement.getRobotPosition().getY(), 0.0);
        yVelocity = -1 * MotorUtils.clamp(yVelocity, -velocityFactor, velocityFactor);
      }

      drivetrainsubsystem.drive(new ChassisSpeeds(xVelocity, yVelocity, rotVelocity));
    }
    else{
      done = true;
    }
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
    VisionMeasurement relativeMeasurement = camerasubsystem.getVisionBotPoseInTargetSpace();
    done = Math.abs(targetX - relativeMeasurement.getRobotPosition().getX()) < tolerance && Math.abs(targetY - relativeMeasurement.getRobotPosition().getY()) < tolerance;
    return done;
  }
}
