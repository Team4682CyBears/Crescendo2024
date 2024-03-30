// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: IntakeAndFeedNoteCommand.java
// Intent: Forms a command to both intake and feed the note to the shooter or dunker. 
// ignores the intake sensor and feeds directtly to the bottom of the shooter or dunker. 
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.common.FeederMode;
import frc.robot.control.Constants;
import frc.robot.control.InstalledHardware;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;

/**
 * Forms a command to feed the note to the shooter or dunker
 * Feeder is run until note is detected or timer has expired
 */
public class IntakeAndFeedNoteCommand extends Command
{
  private IntakeSubsystem intake;
  private FeederSubsystem feeder;
  private Timer timer = new Timer();
  private boolean done = false;
  private FeederMode direction; 
  private RumbleCommand rumbleCommand = null;
  private boolean doRumble = false;
  private boolean rumbleStarted = false;
  
  /** 
  * Creates a new feeder command without rumble. 
  * to be used for auto
  * 
  * @param intakeSubsystem - the intake subsystem
  * @param feederSubsystem - the feeder subsystem
  * @param feederMode - the direction for the feeder
  */
  public IntakeAndFeedNoteCommand(IntakeSubsystem intakeSubsystem, FeederSubsystem feederSubsystem, FeederMode feederMode)
  {
    this.intake = intakeSubsystem;
    this.feeder = feederSubsystem;
    this.direction = feederMode;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, feeder);
  }

  /** 
  * Creates a new feeder command with rumble
  * to be used for UI buttons
  * 
  * @param intakeSubsystem - the intake subsystem
  * @param feederSubsystem - the feeder subsystem
  * @param feederMode - the direction for the feeder
  * @param xboxController - the xbox controller to be used for rumble
  */
  public IntakeAndFeedNoteCommand(IntakeSubsystem intakeSubsystem, FeederSubsystem feederSubsystem, FeederMode feederMode, XboxController xboxController)
  {
    this.intake = intakeSubsystem;
    this.feeder = feederSubsystem;
    this.direction = feederMode;
    this.rumbleCommand = new RumbleCommand(xboxController, Constants.rumbleTimeSeconds);
    this.doRumble = true;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(intake, feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    // set the feeder in the right direction 
    feeder.setFeederMode(direction);
    // intentionally *not* setting intake or feeder speed to 0 here. 
    // If the intake or feeder is currently running, don't stop it. 
    timer.reset();
    timer.start();
    done = false;
    rumbleStarted = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    // rumble once note is detected in the intake
    if (doRumble && !rumbleStarted) {
      if (intake.isNoteDetected()){
        rumbleCommand.schedule(); // want to call this just once
        rumbleStarted = true;
      }
      /*if(InstalledHardware.LEDSInstalled && intake.isNoteDetected()){
      ledSubsystem.RegisterStateAction(orangeBlink, LEDState.OrangeBlink);
    }*/
    }
    // keep feeding until note is detected in shooter
    if (feeder.isShooterNoteDetected()){
      intake.setAllStop();
      feeder.setAllStop();
      done = true;
    } else { // run intake and feeder
      intake.setIntakeSpeed(Constants.intakeSpeed);
      feeder.setFeederSpeed(Constants.feederSpeed);
    }
    if (timer.hasElapsed(Constants.feederTimeoutSeconds))
    {
      intake.setAllStop();
      feeder.setAllStop();
      done = true;
    }
  
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    // Intentionally *not* setting intake or feeder speed to 0 here. 
    // the two conditions above (note detected and timer elasped)
    // already stop the motors. 
    // If this command is interrupted, e.g. by the codriver hitting the button multiple times
    // we don't want to stop and restart the intake or feeder. 
    // There is a default command registered on these sybsystems that stops the motor if no 
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