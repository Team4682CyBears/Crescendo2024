package frc.robot.commands;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.common.FeederMode;
import frc.robot.subsystems.FeederSubsystem;

public class RewindFeederCommand extends Command{
    private FeederSubsystem feeder;
    private FeederMode direction;  
    private boolean done = false;
    private Timer timer = new Timer();
  /** 
  * Creates a new feeder command 
  * 
  * @param feederSubsystem - the feeder subsystem
  * @param feederMode - the direction for the feeder
  */
  public RewindFeederCommand(FeederSubsystem feederSubsystem, FeederMode feederMode)
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
    DataLogManager.log("Starting RewindFeederCommand");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return done;
  }
    
}
