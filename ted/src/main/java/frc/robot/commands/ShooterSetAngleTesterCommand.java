// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: ShooterSetAngleTesterCommand.java
// Intent: Wraps the ShooterSetAngleCommand to accept a double
// supplier for the angle
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import frc.robot.subsystems.ShooterAngleSubsystem;
import java.util.function.DoubleSupplier; 

/**
 * Forms a command to shoot the shooter
 */
public class ShooterSetAngleTesterCommand extends ShooterSetAngleCommand {

  private DoubleSupplier desiredAngleDegreesSupplier; 

  /**
   * Constructor for ShooterSetAngleTesterCommand
   * Will set shooter to desired angle before shooting
   * @param desiredAngleDegreesSupplier
   * @param shooter
   */
  public ShooterSetAngleTesterCommand(DoubleSupplier desiredAngleDegreesSupplier, ShooterAngleSubsystem shooter) {
    super(desiredAngleDegreesSupplier.getAsDouble(), shooter);
    this.desiredAngleDegreesSupplier = desiredAngleDegreesSupplier;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    System.out.println("Initialize ShooteSetAngleTesterCommand");
    super.desiredAngleDegrees = desiredAngleDegreesSupplier.getAsDouble();
    super.initialize();
  }
}