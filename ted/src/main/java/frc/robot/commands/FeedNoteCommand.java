// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: FeedNoteCommand.java
// Intent: Forms a command to feed the note to the shooter or dunker. 
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.common.FeederMode;
import frc.robot.control.Constants;
import frc.robot.subsystems.FeederSubsystem;

/**
 * Forms a command to feed the note to the shooter or dunker
 * Feeder is run until note is detected or timer has expired
 */
public class FeedNoteCommand extends Command
{
  private FeederSubsystem feeder;
  private Timer timer = new Timer();
  private boolean done = false;
  private FeederMode direction; 
  
  /** 
  * Creates a new feeder command 
  * 
  * @param feederSubsystem - the feeder subsystem
  * @param feederMode - the direction for the feeder
  */
  public FeedNoteCommand(FeederSubsystem feederSubsystem, FeederMode feederMode)
  {
    this.feeder = feederSubsystem;
    this.direction = feederMode;
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
    System.out.println("Starting FeedNoteCommand");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    if (feeder.isShooterNoteDetected()){
      feeder.setAllStop();
      done = true;
    } else { // run feeder
      feeder.setFeederSpeed(Constants.feederSpeed);
    }
    if (timer.hasElapsed(Constants.feederTimeoutSeconds))
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
    // the two conditions above (note detected and timer elasped)
    // already stop the motor. 
    // If this command is interrupted, e.g. by the codriver hitting the button multiple times
    // we don't want to stop and restart the intake. 
    // There is a default command registered on this sybsystem that stops the motor if no 
    // other command is running. 
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