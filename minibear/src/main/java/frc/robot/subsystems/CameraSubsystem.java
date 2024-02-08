// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: CameraSubsystem.java
// Intent: Forms the prelminary code for camera train subsystem.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.common.VisionMeasurement;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A class to encapsulate the camera subsystem
 */
public class CameraSubsystem extends SubsystemBase {
  private final double milisecondsInSeconds = 1000.0;
  private final int defaultDoubleArraySize = 7;
  private final int TimestampIndex = 6;
  private final int botPositionXIndex = 0;
  private final int botPositionYIndex = 2;
  private final int botRotationIndex = 5;
  private final int noTagInSightId = -1;
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  /**
   * a constructor for the camera subsystem class
   * @param subsystems - the subsystem collection
   */
  public CameraSubsystem() {
  }

  /**
   * a method that returns a vision measurement. 
   * pose portion of the vision measurement is null if there is no valid measurement. 
   */
  public VisionMeasurement getVisionBotPose(){
    double tagId = table.getEntry("tid").getDouble(0);
    double[] botpose = table.getEntry("botpose").getDoubleArray(new double[defaultDoubleArraySize]);
    Double timestamp = Timer.getFPGATimestamp() - (botpose[TimestampIndex]/milisecondsInSeconds);
    Translation2d botTranslation = new Translation2d(botpose[botPositionXIndex], botpose[botPositionYIndex]);
    Rotation2d botYaw = Rotation2d.fromDegrees(botpose[botRotationIndex]);
    Pose2d realRobotPosition = new Pose2d(botTranslation, botYaw);

    if (tagId == noTagInSightId){
      return new VisionMeasurement(null, 0.0);
    }
    else{
      return new VisionMeasurement(realRobotPosition, timestamp);
    }
  }

  /**
   * a method that returns a vision measurement. 
   * pose portion of the vision measurement is null if there is no valid measurement. 
   */
  public Pose2d getVisionBotPoseInTargetSpace(){
    double tagId = table.getEntry("tid").getDouble(0);
    double[] botpose = table.getEntry("botpose_targetspace").getDoubleArray(new double[defaultDoubleArraySize]);
    Translation2d botTranslation = new Translation2d(botpose[botPositionXIndex], botpose[botPositionYIndex]);
    Rotation2d botYaw = Rotation2d.fromDegrees(botpose[botRotationIndex]);
    Pose2d realRobotPosition = new Pose2d(botTranslation, botYaw);

    if (tagId == noTagInSightId){
      return null;
    }
    else{
      return realRobotPosition;
    }
  }

  /**
   * A method to run during periodic for the camera subsystem
   * it reads tags and updates the estimated position in the drivetrain subsystem
   */
  @Override
  public void periodic() {
    SmartDashboard.putNumber("relative X", this.getVisionBotPoseInTargetSpace().getX());
    SmartDashboard.putNumber("relative Y", this.getVisionBotPoseInTargetSpace().getY());
  }
}