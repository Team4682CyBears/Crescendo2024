// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: CameraSubsystem.java
// Intent: Forms the prelminary code for the camera subsystem
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;;

/**
 * A class to encapsulate the camera subsystem
 */
public class CameraSubsystem extends SubsystemBase {
  private final double milisecondsInSeconds = 1000.0;
  private final int defaultDoubleArraySize = 7;
  private final int TimestampIndex = 6;
  private final int botPositionXIndex = 0;
  private final int botPositionYIndex = 2;
  private final int botRotationIndex=  5;
  private final int noTageInSightId = -1;
  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

  /**
   * a constructor for the camera subsystem class
   * @param subsystems - the subsystem collection
   */
  public CameraSubsystem() {
  }

  public VisionMeasurement getVisionBotPose(){
    double tagId = table.getEntry("tid").getDouble(0);
    double[] botpose = table.getEntry("botpose").getDoubleArray(new double[defaultDoubleArraySize]);
    Double timestamp = Timer.getFPGATimestamp() - (botpose[TimestampIndex]/milisecondsInSeconds);
    Pose2d realRobotPosition = new Pose2d(new Translation2d(botpose[botPositionXIndex], botpose[botPositionYIndex]), Rotation2d.fromDegrees(botpose[botRotationIndex]));

    if (tagId == noTageInSightId){
      return new VisionMeasurement(null, 0.0);
    }
    else{
      return new VisionMeasurement(realRobotPosition, timestamp);
    }
  }

  public double getTagId(){
    return table.getEntry("tid").getDouble(0);
  }

}
