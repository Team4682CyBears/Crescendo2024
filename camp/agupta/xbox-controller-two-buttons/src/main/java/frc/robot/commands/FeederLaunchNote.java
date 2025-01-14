package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.common.FeederMode;
import frc.robot.control.Constants;
import frc.robot.subsystems.FeederSubsystem;

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
  * @param timeoutTime - the duration you want the feeder to run for
  */
  public FeederLaunchNote(FeederSubsystem feederSubsystem, FeederMode feederMode, double timeoutTime)
  {
    this.feeder = feederSubsystem;
    this.direction = feederMode;
    this.timeoutTime = timeoutTime;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(feeder);
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
