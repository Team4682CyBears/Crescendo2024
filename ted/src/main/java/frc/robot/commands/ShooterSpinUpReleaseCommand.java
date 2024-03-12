// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: ShooterSpinUpReleaseCommand.java
// Intent: Forms a command to spin up the shooter outake motors 
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterOutfeedSubsystem;

/**
 * Creates a new command that will run gracefully wind down shooter outfeed motors
 */
public class ShooterSpinUpReleaseCommand extends Command
{
  /** 
  * Creates a new command that will run gracefully wind down shooter outfeed motors
  * 
  * @param shooterSubsystem - the shooter outfeed subsystem
  */
  public ShooterSpinUpReleaseCommand(ShooterOutfeedSubsystem shooterSubsystem)
  {
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return true;
  }
}