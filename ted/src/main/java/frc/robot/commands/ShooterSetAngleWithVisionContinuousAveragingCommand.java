// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: ShooterSetAngleWithVisionContinuousAveragingCommand.java
// Intent: Wraps the ShooterSetAngleCommand to set angle with vision
// averages last three vision estiamtes to get more accurate distance
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import java.util.ArrayList;

import frc.robot.common.DistanceMeasurement;
import frc.robot.common.ShooterAngleHelpers;
import frc.robot.control.Constants;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.ShooterAngleSubsystem;

/**
 * Forms a command to set the angle of the shooter from the caemra. 
 * averages last three vision estiamtes to get more accurate distance
 * It keeps running until canceled
 * Appropriate to be used for binding to a UI button that is triggered onTrue, for example
 */
public class ShooterSetAngleWithVisionContinuousAveragingCommand extends ShooterSetAngleCommand {

  private double desiredAngleDegrees;
  private ShooterAngleSubsystem shooterAngleSubsystem;
  private CameraSubsystem cameraSubsystem;
  private static final int distanceListMaxSize = 3;
  private ArrayList<Double> recentDistances = new ArrayList<Double>();
  private int desiredValidSamples = 3;

  /**
   * Constructor for ShooterSetAngleWithVisionCommand
   * Will set shooter to desired angle before shooting
   * @param cameraSubsystem
   * @param shooterSubsystem
   */
  public ShooterSetAngleWithVisionContinuousAveragingCommand(CameraSubsystem cameraSubsystem, ShooterAngleSubsystem shooterAngleSubsystem) {
    // start out wanting current shooter angle
    super(shooterAngleSubsystem.getAngleDegrees(), shooterAngleSubsystem);
    this.shooterAngleSubsystem = shooterAngleSubsystem;
    this.cameraSubsystem = cameraSubsystem;
    this.desiredAngleDegrees = this.shooterAngleSubsystem.getAngleDegrees();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // start out wanting current shooter angle
    desiredAngleDegrees = this.shooterAngleSubsystem.getAngleDegrees();
    super.desiredAngleDegrees = desiredAngleDegrees;
    recentDistances = new ArrayList<Double>(); // clear distance history
    super.initialize();
  }

  @Override
  public void execute() {
    storeDistance();
    // wait until there are the right number of samples to average 
    if (recentDistances.size() >= desiredValidSamples){
      desiredAngleDegrees = ShooterAngleHelpers.shooterAngleFromDistance(getAverageDistance());
      super.desiredAngleDegrees = desiredAngleDegrees;
    }
     // else stay at previous angle
    super.execute();
  }

  @Override
  public boolean isFinished() {
    // this command keeps running until canceled
    return false;
  }

   /**
   * Method that will store valid distance
   */
  private void storeDistance(){
    DistanceMeasurement distanceMeasurement = cameraSubsystem.getDistanceFromTag(Constants.speakerBlueTagID, Constants.spekaerRedTagID);
    if (distanceMeasurement.getIsValid()){
      this.recentDistances.add(distanceMeasurement.getDistanceMeters());
    }
    while(this.recentDistances.size() > distanceListMaxSize)
    {
      recentDistances.remove(0);
    }
  }

  /**
   * A method to return the average distance
   * @return distance
   */
  private double getAverageDistance(){
    double distanceSum = 0;
    for(int i = 0; i < this.recentDistances.size(); i++){
      distanceSum += recentDistances.get(i);
    }
    double averageDistance = distanceSum/this.recentDistances.size();
    // DataLogManager.log("Calculated avearge distance of " + averageDistance + " from distances of " + recentDistances);
    return averageDistance;
  }

}