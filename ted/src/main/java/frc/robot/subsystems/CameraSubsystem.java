// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: CameraSubsystem.java
// Intent: Forms the prelminary code for the camera subsystem.
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
import frc.robot.common.DistanceMeasurement;

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
  private NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  /**
   * a constructor for the camera subsystem class
   */
  public CameraSubsystem() {
  }

  /**
   * a method that returns a vision measurement. 
   * pose portion of the vision measurement is null if there is no valid measurement. 
   * @return a vision measurement of the bot pose in field space
   */
  public VisionMeasurement getVisionBotPose(){
    double tagId = table.getEntry("tid").getDouble(0);

    if (tagId == noTagInSightId){
      return new VisionMeasurement(null, 0.0);
    }
    else{
      double[] botpose = this.table.getEntry("botpose").getDoubleArray(new double[this.defaultDoubleArraySize]);
      Double timestamp = Timer.getFPGATimestamp() - (botpose[this.TimestampIndex]/this.milisecondsInSeconds);
      Translation2d botTranslation = new Translation2d(botpose[this.botPositionXIndex], botpose[this.botPositionYIndex]);
      Rotation2d botYaw = Rotation2d.fromDegrees(botpose[this.botRotationIndex]);
      Pose2d realRobotPosition = new Pose2d(botTranslation, botYaw);
      return new VisionMeasurement(realRobotPosition, timestamp);
    }
  }

  /**
   * a method that returns the tag id of the current viewed tag
   * @return double of the current in view tag id
   */
  public double getTagId(){
    return table.getEntry("tid").getDouble(0);
  }

  /**
   * a method that returns the robots distance from one of the given tags
   * @param blueTId
   * @param redTId
   * @return a ditance measuremtn of the distance, with false if the tag is invalid
   */
  public DistanceMeasurement getDistanceFromTag(double blueTId, double redTId){
    DistanceMeasurement measurement = new DistanceMeasurement(false, 0.0);
    if((getTagId() == blueTId || getTagId() == redTId) && getVisionBotPoseInTargetSpace() != null){
      measurement.setIsValid(true);

      double xDistance = getVisionBotPoseInTargetSpace().getTranslation().getX();
      double yDistance = getVisionBotPoseInTargetSpace().getTranslation().getY();
      double totalDistance = Math.sqrt(Math.pow(xDistance, 2) + Math.pow(yDistance, 2));
      measurement.setDistanceMeteres(totalDistance);
    }
    return measurement;
  }

  /**
   * a method that returns a pose2d of the robot pose in target space. 
   * pose portion of the vision measurement is null if there is no valid measurement. 
   */
  public Pose2d getVisionBotPoseInTargetSpace(){
    double tagId = table.getEntry("tid").getDouble(0);
    double[] botpose = this.table.getEntry("botpose_targetspace").getDoubleArray(new double[this.defaultDoubleArraySize]);
    Translation2d botTranslation = new Translation2d(botpose[this.botPositionXIndex], botpose[this.botPositionYIndex]);
    Rotation2d botYaw = Rotation2d.fromDegrees(botpose[this.botRotationIndex]);
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
    Pose2d dm = getVisionBotPoseInTargetSpace();
    if(dm != null){
      SmartDashboard.putNumber("relative X", dm.getX());
      SmartDashboard.putNumber("relative Y", dm.getY());
    } 

    DistanceMeasurement measurement = this.getDistanceFromTag(7.0, 4.0);
    double putter = 0.0;
    if(measurement.getIsValid()){
      putter = measurement.getDistanceMeters();
    }
    SmartDashboard.putNumber("distance from tag", putter);
    SmartDashboard.putNumber("vison angle", -4.65*putter + 51.6);

  }
}