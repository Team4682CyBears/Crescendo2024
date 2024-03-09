// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: ShooterSpinUpForeverCommand.java
// Intent: Forms a command to spin up the shooter outake motors 
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.ShooterOutfeedSubsystem;

/**
 * Forms a command to spin up shooter forever the command will run 
 * until it is interrupted by another command that requires the shooter
 * outfeed motor subsystem
 */
public class ShooterSpinUpForeverCommand extends Command
{
  private ShooterOutfeedSubsystem shooterOutfeed;
  private FeederSubsystem feeder;
  private boolean noteRequired = false;
  private boolean done = false;
  private DoubleSupplier targetShooterSpeed;
  
  /** 
  * Creates a new command that will run the shooter outfeed motors until it is
  * interrupted by another command that requires the shooter outfeed motor subsystem
  * 
  * @param shooterSubsystem - the shooter outfeed subsystem
  */
  public ShooterSpinUpForeverCommand(ShooterOutfeedSubsystem shooterSubsystem,
                                     FeederSubsystem feederSubsystem,
                                     DoubleSupplier targetShooterSpeedSupplier,
                                     boolean requireNotePresent)
  {
    this.shooterOutfeed = shooterSubsystem;
    addRequirements(shooterOutfeed);

    // NOTE THE FEEDER MUST ONLY EVER BE USED IN READ SITUATIONS FROM WITHIN THIS CLASS!
    // DO NOT ADD FEEDER AS A REQUIRED SUBSYSTEM UP TO SCHEDULER!!!
    this.feeder = feederSubsystem;
    
    this.targetShooterSpeed = targetShooterSpeedSupplier;
    this.noteRequired = requireNotePresent;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
      done = this.shouldStopSpinning();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    if(!done){
        double targetSpeed = targetShooterSpeed.getAsDouble();
        shooterOutfeed.setShooterVelocityLeft(targetSpeed);
        shooterOutfeed.setShooterVelocityRight(targetSpeed);
        done = this.shouldStopSpinning();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
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

  /**
   * A method that will determine if we should stop spinning
   * @return false if note is not required, false when note is present and only true when note is not present and also its presence is required
   */
  private boolean shouldStopSpinning() {
    boolean result = false;
    if(this.noteRequired) {
      result = !feeder.isShooterNoteDetected();
    }
    return result;
  }
}