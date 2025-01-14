// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: ShooterSpinUpCommand.java
// Intent: Forms a command to spin up the shooter outake motors 
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.control.Constants;
import frc.robot.subsystems.ShooterOutfeedSubsystem;

/**
 * Forms a command to intake the note
 * Intake is run until note is detected or timer has expired
 */
public class ShooterSpinUpCommand extends Command
{
  private ShooterOutfeedSubsystem shooterOutfeed;
  private Timer timer = new Timer();
  private boolean done = false;
  private DoubleSupplier desiredSpeedRpmSupplier;
  
  /** 
  * Creates a new shooter spin up command 
  * 
  * @param shooterSubsystem - the shooter outfeed subsystem
  * @param desiredSpeedRpmSupplier - the desired speed supplier
  */
  public ShooterSpinUpCommand(ShooterOutfeedSubsystem shooterSubsystem, DoubleSupplier desiredSpeedRpmSupplier)
  {
    this.shooterOutfeed = shooterSubsystem;
    this.desiredSpeedRpmSupplier = desiredSpeedRpmSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterOutfeed);
  }
  
  /** 
  * Creates a new shooter spin up command 
  * 
  * @param shooterSubsystem - the shooter outfeed subsystem
  */
  public ShooterSpinUpCommand(ShooterOutfeedSubsystem shooterSubsystem)
  {
    this(shooterSubsystem, () -> Constants.shooterDefaultSpeedRpm);
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
    shooterOutfeed.setShooterVelocity(desiredSpeedRpmSupplier.getAsDouble());
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
    // the timer elapsed conditions above 
    // already stop the motor. 
    // If this command is interrupted, e.g. by the codriver hitting the button multiple times
    // we don't want to stop and restart the shooter. 
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