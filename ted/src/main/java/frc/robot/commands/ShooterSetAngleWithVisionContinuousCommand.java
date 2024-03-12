// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: ShooterSetAngleWithVisionCommand.java
// Intent: Wraps the ShooterSetAngleWithVision command to set angle with vision
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import frc.robot.common.DistanceMeasurement;
import frc.robot.common.ShooterAngleHelpers;
import frc.robot.control.Constants;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.ShooterAngleSubsystem;

/**
 * Forms a command to set the angle of the shooter from the caemra. 
 * It keeps running until canceled
 * Appropriate to be used for binding to a UI button that is triggered onTrue, for example
 */
public class ShooterSetAngleWithVisionContinuousCommand extends ShooterSetAngleCommand {

  private double desiredAngleDegrees;
  private ShooterAngleSubsystem shooterAngleSubsystem;
  private CameraSubsystem cameraSubsystem;

  /**
   * Constructor for ShooterSetAngleWithVisionCommand
   * Will set shooter to desired angle before shooting
   * @param cameraSubsystem
   * @param shooterSubsystem
   */
  public ShooterSetAngleWithVisionContinuousCommand(CameraSubsystem cameraSubsystem, ShooterAngleSubsystem shooterAngleSubsystem) {
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
    super.initialize();
  }

  @Override
  public void execute() {
    DistanceMeasurement distanceMeasurement = cameraSubsystem.getDistanceFromTag(Constants.speakerBlueTagID, Constants.spekaerRedTagID);
    if (distanceMeasurement.getIsValid()){
      desiredAngleDegrees = ShooterAngleHelpers.shooterAngleFromDistance(distanceMeasurement.getDistanceMeters());
      super.desiredAngleDegrees = desiredAngleDegrees;
    } // else stay at previous angle
    super.execute();
  }

  @Override
  public boolean isFinished() {
    // this command keeps running until canceled
    return false;
  }

}