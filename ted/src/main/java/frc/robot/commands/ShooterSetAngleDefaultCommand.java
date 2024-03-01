// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: ShooterSetAngleTesterCommand.java
// Intent: Wraps the ShooterSetAngleDefaultCommand to accept a controller axis
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import frc.robot.common.CoDriverMode;
import frc.robot.common.MotorUtils;
import frc.robot.control.Constants;
import frc.robot.control.ManualInputInterfaces;
import frc.robot.subsystems.ShooterAngleSubsystem;
import java.util.function.DoubleSupplier; 

/**
 * Forms a command to shoot the shooter
 */
public class ShooterSetAngleDefaultCommand extends ShooterSetAngleCommand {

  private DoubleSupplier desiredAngleDegreesSupplier; 
  private double desiredAngleDegrees;
  private ShooterAngleSubsystem shooter;
  private ManualInputInterfaces manual;
  private CoDriverMode currentDriverMode = CoDriverMode.AmpScore;

  /**
   * Constructor for ShooterSetAngleTesterCommand
   * Will set shooter to desired angle before shooting
   * @param desiredAngleDegreesSupplier
   * @param shooter
   */
  public ShooterSetAngleDefaultCommand(
    DoubleSupplier desiredAngleDegreesSupplier,
    ShooterAngleSubsystem shooter,
    ManualInputInterfaces manualInput) {
    // start out wanting current shooter angle
    super(shooter.getAngleDegrees(), shooter);
    this.shooter = shooter;
    this.desiredAngleDegreesSupplier = desiredAngleDegreesSupplier;
    this.desiredAngleDegrees = this.shooter.getAngleDegrees();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    currentDriverMode = manual.getCoDriverMode();
    if((currentDriverMode == CoDriverMode.AmpScore || currentDriverMode == CoDriverMode.SpeakerScore)){
      // start out wanting current shooter angle
      desiredAngleDegrees = this.shooter.getAngleDegrees();
      super.desiredAngleDegrees = desiredAngleDegrees;
      super.initialize();
    }
    // skip init when not in proper mode
    else {
      super.done = true;
    }
  }

  @Override
  public void execute() {
    if((currentDriverMode == CoDriverMode.AmpScore || currentDriverMode == CoDriverMode.SpeakerScore)){
      desiredAngleDegrees += desiredAngleDegreesSupplier.getAsDouble();
      desiredAngleDegrees = MotorUtils.clamp(desiredAngleDegrees, Constants.shooterAngleDegreesMinimum, Constants.shooterAngleDegreesMaximum);
      super.desiredAngleDegrees = desiredAngleDegrees;
      super.execute();
    }
    // skip execute when not in proper mode
    else {
      super.done = true;
    }
  }

}