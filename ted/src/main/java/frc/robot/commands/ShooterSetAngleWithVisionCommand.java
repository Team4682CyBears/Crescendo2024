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
import frc.robot.common.MotorUtils;
import frc.robot.control.Constants;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.ShooterAngleSubsystem;

/**
 * Forms a command to shoot the shooter
 */
public class ShooterSetAngleWithVisionCommand extends ShooterSetAngleCommand {

  private double desiredAngleDegrees;
  private ShooterAngleSubsystem shooterAngleSubsystem;
  private CameraSubsystem cameraSubsystem;
  //We use these linear functions to determine which angle the shooter should be at, given distance
  //There are two functions becuase it is a piecewise function
  private double farSlope = -4.65;
  private double farOffset = 51.6;
  private double closeSlope = -17.9;
  private double closeOffset = 79.1;
  private double nearFarBreakpoint = 2.0;

  /**
   * Constructor for ShooterSetAngleWithVisionCommand
   * Will set shooter to desired angle before shooting
   * @param cameraSubsystem
   * @param shooterSubsystem
   */
  public ShooterSetAngleWithVisionCommand(CameraSubsystem cameraSubsystem, ShooterAngleSubsystem shooterAngleSubsystem) {
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
    DistanceMeasurement distanceMeasurement = cameraSubsystem.getDistanceFromTag(7.0, 4.0);
    double slope = 0.0;
    double offset = 0.0;
    if (distanceMeasurement.getIsValid()){
      if(distanceMeasurement.getDistanceMeters() < nearFarBreakpoint){
        slope = closeSlope;
        offset = closeOffset;
      }
      else{
        slope = farSlope;
        offset = farOffset;
      }
      desiredAngleDegrees = (slope*distanceMeasurement.getDistanceMeters()) + offset;
      desiredAngleDegrees = MotorUtils.clamp(desiredAngleDegrees, Constants.shooterAngleMinDegrees, Constants.shooterAngleMaxDegrees);
      super.desiredAngleDegrees = desiredAngleDegrees;
    } // else stay at previous angle
    super.execute();
  }
}