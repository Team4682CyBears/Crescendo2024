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

import frc.robot.common.FeederMode;
import frc.robot.control.Constants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.TalonShooterSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier; 

/**
 * Forms a command to shoot the shooter
 */
public class ShooterSetAngleTesterCommand extends Command {

  private TalonShooterSubsystem shooter;
  private DoubleSupplier desiredAngleDegreesSupplier; 
  private Command shooterSetAngleCommand;

  /**
   * Constructor for ShooterSetAngleTesterCommand
   * Will set shooter to desired angle before shooting
   * @param desiredAngleDegreesSupplier
   * @param shooter
   */
  public ShooterSetAngleTesterCommand(DoubleSupplier desiredAngleDegreesSupplier, TalonShooterSubsystem shooter) {
    this.desiredAngleDegreesSupplier = desiredAngleDegreesSupplier;
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    // ShooterSetAngleCommand will set requirements
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooterSetAngleCommand = new ShooterSetAngleCommand(desiredAngleDegreesSupplier.getAsDouble(), shooter);
    shooterSetAngleCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSetAngleCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterSetAngleCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooterSetAngleCommand.isFinished();
  }

}