// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: FeederSubsystem.java
// Intent: Forms a command to run the bag motor.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

// declare package to contain current class
package frc.robot.commands;

// import wpi libraries
//import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;

// import local classes
import frc.robot.common.BagMode;
import frc.robot.subsystems.BagSubsystem;

public class RunBagCommand extends Command
{
  private BagSubsystem bag;
  private BagMode direction; 
  private boolean done = false;
 
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
    bag.setBagMode(direction);
    done = false;
    //DataLogManager.log("Starting RunBagCommand");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){}

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
