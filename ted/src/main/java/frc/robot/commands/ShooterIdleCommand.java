// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: ShooterIdleCommand.java
// Intent: Forms a command to spin up the shooter outake motors to idle speed
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.common.ShooterOutfeedSpeed;
import frc.robot.control.Constants;
import frc.robot.control.ManualInputInterfaces;
import frc.robot.subsystems.ShooterOutfeedSubsystem;

/**
 * Forms a command to spin outtake motors to idle currently selected speed
 */
public class ShooterIdleCommand extends Command
{
  private ShooterOutfeedSubsystem shooterOutfeed;
  private ManualInputInterfaces input; 
  private double shotSpeed = 0;
  
  /** 
  * Creates a new shooter idle speed command 
  * @param shooterSubsystem - the shooter outfeed subsystem
  * @param manualInput - the manual input interface where current idle speed setting is stored
  */
  public ShooterIdleCommand(
    ShooterOutfeedSubsystem shooterSubsystem,
    ManualInputInterfaces manualInput)
  {
    this.shooterOutfeed = shooterSubsystem;
    this.input = manualInput;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterOutfeed);
  }

/**
   * A method to translate shooter positions into degrees
   * @param position
   * @return speed of target motors
   */
  public static double getOutfeedSpeedEnumInTargetMotorRpm(ShooterOutfeedSpeed outfeedSpeed){
    double speed = Constants.shooterSpeedRpmStopped;
    switch (outfeedSpeed) {
      case Stopped:
        speed = Constants.shooterSpeedRpmStopped;
        break;
      case SpeakerIdle:
        speed = Constants.shooterSpeedRpmSpeakerIdle;
        break;
      case SpeakerCloseDistance:
        speed = Constants.shooterSpeedRpmSpeakerCloseDistance;
        break;
      case SpeakerPodiumDistance:
        speed = Constants.shooterSpeedRpmSpeakerPodiumDistance;
        break;
      case SpeakerRedlineDistance:
        speed = Constants.shooterSpeedRpmSpeakerRedlineDistance;
        break;
      case AmpIdle:
        speed = Constants.shooterSpeedRpmAmpIdle;
        break;
      case AmpLow:
        speed = Constants.shooterSpeedRpmAmpLow;
        break;
      case AmpMedium:
        speed = Constants.shooterSpeedRpmAmpMedium;
        break;
      case AmpHigh:
        speed = Constants.shooterSpeedRpmAmpHigh;
        break;
      default:
        speed = Constants.shooterSpeedRpmStopped;
        break;
    }
    return speed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    this.shotSpeed = 
    ShooterIdleCommand.getOutfeedSpeedEnumInTargetMotorRpm(input.getShooterOutfeedSpeed());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    shooterOutfeed.setShooterVelocityLeft(this.shotSpeed);
    shooterOutfeed.setShooterVelocityRight(this.shotSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return true;
  }
}