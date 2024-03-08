// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: ShooterSpinUpForeverCommand.java
// Intent: Forms a command to spin up the shooter outake motors 
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.control.Constants;
import frc.robot.subsystems.ShooterOutfeedSubsystem;

/**
 * Forms a command to spin up shooter forever the command will run 
 * until it is interrupted by another command that requires the shooter
 * outfeed motor subsystem
 */
public class ShooterSpinUpForeverCommand extends Command
{
  private ShooterOutfeedSubsystem shooterOutfeed;
  private boolean done = false;
  
  /** 
  * Creates a new command that will run the shooter outfeed motors until it is
  * interrupted by another command that requires the shooter outfeed motor subsystem
  * 
  * @param shooterSubsystem - the shooter outfeed subsystem
  */
  public ShooterSpinUpForeverCommand(ShooterOutfeedSubsystem shooterSubsystem)
  {
    this.shooterOutfeed = shooterSubsystem;
    addRequirements(shooterOutfeed);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    done = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    if(!done){
        shooterOutfeed.setShooterVelocityLeft(Constants.shooterLeftDefaultSpeedRpm);
        shooterOutfeed.setShooterVelocityRight(Constants.shooterRightDefaultSpeedRpm);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    if(interrupted)
    {
        done = true;      
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return done;
  }
}