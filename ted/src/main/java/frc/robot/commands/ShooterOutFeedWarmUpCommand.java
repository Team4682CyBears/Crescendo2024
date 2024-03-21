// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: ShooterFreeSpinUpCommand.java
// Intent: Forms a command to spin up the shooter outake motors 
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.control.Constants;
import frc.robot.subsystems.ShooterOutfeedSubsystem;

/**
 * Forms a command to warm up the shooter
 * Intake is run until note is detected or timer has expired
 */
public class ShooterOutFeedWarmUpCommand extends Command
{
  private ShooterOutfeedSubsystem shooterOutfeed;
  private double warmupSpeedRpm = 0.0;
  private Timer timer = new Timer();
  private boolean done = false;
  
  /** 
  * Creates a new warm up the shooter
  * 
  * @param shooterSubsystem - the shooter outfeed subsystem
  */
  public ShooterOutFeedWarmUpCommand(
    ShooterOutfeedSubsystem shooterSubsystem,
    double targetSpeedRpm)
  {
    this.shooterOutfeed = shooterSubsystem;
    this.warmupSpeedRpm = targetSpeedRpm;
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
    System.out.println("Starting ShooterOutFeedWarmUpCommand");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    shooterOutfeed.setShooterVelocity(warmupSpeedRpm);
    if (timer.hasElapsed(Constants.shooterSpinUpTimeoutSeconds))
    {
        done = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    // Intentionally *not* setting shooter speed to 0 here. 
    // we want the mechanism to keep spinning so that any inertia is not wasted
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