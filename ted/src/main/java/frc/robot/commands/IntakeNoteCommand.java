// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: IntakeNoteCommand.java
// Intent: Forms a command to intake the note. 
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.control.Constants;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Forms a command to intake the note
 * Intake is run until note is detected or timer has expired
 */
public class IntakeNoteCommand extends Command
{
  private IntakeSubsystem intake;
  private Timer timer = new Timer();
  private boolean done = false;
  
  /** 
  * Creates a new intake command 
  * 
  * @param intakeSubsystem - the intake subsystem
  */
  public IntakeNoteCommand(IntakeSubsystem intakeSubsystem)
  {
    this.intake = intakeSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    // intentionally *not* setting intake speed to 0 here. 
    // If the intake is currently running, don't stop it. 
    timer.reset();
    timer.start();
    done = false;
    DataLogManager.log("Starting IntakeNoteCommand");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    if (intake.isNoteDetected()){
      intake.setAllStop();
      done = true;
    } else { // run intake
      intake.setIntakeSpeed(Constants.intakeSpeed);
    }
    if (timer.hasElapsed(Constants.intakeTimeoutSeconds))
    {
      intake.setAllStop();
      done = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    // Intentionally *not* setting intake speed to 0 here. 
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