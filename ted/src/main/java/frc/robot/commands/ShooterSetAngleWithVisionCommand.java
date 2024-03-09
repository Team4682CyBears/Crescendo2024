// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: ShooterSetAngleTesterCommand.java
// Intent: Wraps the ShooterSetAngleDefaultCommand to accept a controller axis
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
  private ShooterAngleSubsystem shooter;
  private CameraSubsystem cameraSubsystem;
  private double farSlope = -4.65;
  private double farOffset = 51.6;
  private double closeSlope = -17.9;
  private double closeOffset = 79.1;
  private double nearFarBreakpoint = 2.0;

  /**
   * Constructor for ShooterSetAngleTesterCommand
   * Will set shooter to desired angle before shooting
   * @param desiredAngleDegreesSupplier
   * @param shooter
   */
  public ShooterSetAngleWithVisionCommand(CameraSubsystem cameraSubsystem, ShooterAngleSubsystem shooter) {
    // start out wanting current shooter angle
    super(shooter.getAngleDegrees(), shooter);
    this.shooter = shooter;
    this.cameraSubsystem = cameraSubsystem;
    this.desiredAngleDegrees = this.shooter.getAngleDegrees();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // start out wanting current shooter angle
    desiredAngleDegrees = this.shooter.getAngleDegrees();
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
      desiredAngleDegrees = (slope*cameraSubsystem.getDistanceFromTag(7.0, 4.0).getDistanceMeters()) + offset;
      desiredAngleDegrees = MotorUtils.clamp(desiredAngleDegrees, Constants.shooterAngleMinDegrees, Constants.shooterAngleMaxDegrees);
      super.desiredAngleDegrees = desiredAngleDegrees;
    } // else stay at previous angle
    super.execute();
  }
}