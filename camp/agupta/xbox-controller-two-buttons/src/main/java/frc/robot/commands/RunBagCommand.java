package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.common.BagMode;
import frc.robot.control.Constants;
import frc.robot.subsystems.BagSubsystem;

public class RunBagCommand extends Command
{
  private BagSubsystem bag;
  private BagMode direction; 
  private boolean done = false;
  private Timer timer = new Timer();
 
  /** 
  * Moves bag motor forward
  * @param bagSubsystem - the bag subsystem
  * @param bagMode - the direction for the bag
  */

  public RunBagCommand(BagSubsystem bagSubsystem, BagMode bagMode)
  {
    this.bag = bagSubsystem;
    this.direction = bagMode;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(bag);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    // set the bag in the right direction 
    bag.setBagMode(direction);
    // intentionally *not* setting bag speed to 0 here. 
    // If the bag is currently running, don't stop it. 
    timer.reset();
    timer.start();
    done = false;
    DataLogManager.log("Starting RewindbagCommand");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    if(!done) {
      bag.setBagMode(direction);
    }
    //if (timer.hasElapsed(Constants.bagSpeed))
    //{
      //bag.setAllStop();
      //done = true;
    //}
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    // Intentionally *not* setting bag speed to 0 here. 
    if(interrupted)
    {
      bag.setAllStop();
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
