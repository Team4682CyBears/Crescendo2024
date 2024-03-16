// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: FeedNoteCommand.java
// Intent: Forms a command to shoot a note assuming the shooter is 
// already at the right angle and speed and does not require the shooter outfeed subsystem
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.common.FeederMode;
import frc.robot.control.Constants;
import frc.robot.subsystems.FeederSubsystem;

/**
 * Forms a command to feed the note to the shooter
 * Feeder is run until timer has expired
 */
public class FeederLaunchNote extends Command
{
  private FeederSubsystem feeder;
  private Timer timer = new Timer();
  private boolean done = false;
  private FeederMode direction; 
  private double timeoutTime;
  
  /** 
  * Creates a new feeder launch note command
  * @param feederSubsystem - the feeder subsystem
  * @param feederMode - the direction for the feeder
  */
  public FeederLaunchNote(FeederSubsystem feederSubsystem, FeederMode feederMode, double timeoutTime)
  {
    this.feeder = feederSubsystem;
    this.direction = feederMode;
    this.timeoutTime = timeoutTime;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    // set the feeder in the right direction 
    feeder.setFeederMode(direction);
    // intentionally *not* setting feeder speed to 0 here. 
    // If the feeder is currently running, don't stop it. 
    timer.reset();
    timer.start();
    done = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    feeder.setFeederSpeed(Constants.feederSpeed);
    if (timer.hasElapsed(timeoutTime))
    {
      feeder.setAllStop();
      done = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    // Intentionally *not* setting feeder speed to 0 here. 
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