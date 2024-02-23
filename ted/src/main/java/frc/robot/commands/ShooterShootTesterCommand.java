// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: ShooterShootTesterCommand.java
// Intent: Wraps the ShooterShootCommand to allow a double supplier 
// for the left and right shooter speeds.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.TalonShooterSubsystem;

import java.util.function.DoubleSupplier; 

/**
 * Wraps the ShooterShootCommand to allow a double supplier 
 * for the left and right shooter speeds.
 */
public class ShooterShootTesterCommand extends Command {

  private DoubleSupplier desiredLeftSpeedRpmSupplier; 
  private DoubleSupplier desiredRightSpeedRpmSupplier;  
  private Command shooterShootCommand;
  private TalonShooterSubsystem shooter;
  private FeederSubsystem feeder;

  /**
   * Constructor for ShooterShootTesterCommand
   * Will set shooter to desired angle and speeds before shooting
   * assumes already at the right angle
   * @param desiredLeftSpeedRpmSupplier
   * @param desiredRightSpeedRpmSupplier
   * @param shooter
   * @param feeder
   */
  public ShooterShootTesterCommand(
      DoubleSupplier desiredLeftSpeedRpmSupplier,
      DoubleSupplier desiredRightSpeedRpmSupplier,
      TalonShooterSubsystem shooter,
      FeederSubsystem feeder) {
    this.desiredLeftSpeedRpmSupplier = desiredLeftSpeedRpmSupplier;
    this.desiredRightSpeedRpmSupplier = desiredRightSpeedRpmSupplier; 
    this.shooter = shooter;
    this.feeder = feeder;
    // Use addRequirements() here to declare subsystem dependencies.
    // ShooterShootCommand will set the requirements
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // get the speeds and create the ShooterShootCommand
    shooterShootCommand = new ShooterShootCommand(
      desiredLeftSpeedRpmSupplier.getAsDouble(), 
      desiredRightSpeedRpmSupplier.getAsDouble(),
      shooter,
      feeder);
    shooterShootCommand.initialize();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterShootCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooterShootCommand.end(interrupted);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooterShootCommand.isFinished();
  }

}