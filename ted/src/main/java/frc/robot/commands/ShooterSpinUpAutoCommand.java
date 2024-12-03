// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: AutoShooterSpinUpCommand.java
// Intent: Forms a command to spin up the shooter outake motors during auto
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.control.Constants;
import frc.robot.subsystems.ShooterOutfeedSubsystem;

/**
 * Forms a command rev the shooter
 * shooter revs for 15seconds (the auto duration)
 */
public class ShooterSpinUpAutoCommand extends Command
{
  private ShooterOutfeedSubsystem shooterOutfeed;
  private Timer timer = new Timer();
  private boolean done = false;
  
  /** 
  * creates a new auto shooter spin up command
  * @param shooterSubsystem - the shooter outfeed subsystem
  */
  public ShooterSpinUpAutoCommand(ShooterOutfeedSubsystem shooterSubsystem)
  {
    this.shooterOutfeed = shooterSubsystem;
    addRequirements(shooterOutfeed);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    // intentionally *not* setting shooter speed to 0 here. 
    // If the shooter is currently running, don't stop it. 
    timer.reset();
    timer.start();
    done = false;
    DataLogManager.log("Starting SpinUpShooterCommand");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    shooterOutfeed.setShooterVelocity(Constants.shooterMaxRpm);
    if (timer.hasElapsed(Constants.autoShooterSpinUpTimeoutSeconds))
    {
      shooterOutfeed.setAllStop();
      done = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    shooterOutfeed.setAllStop();
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