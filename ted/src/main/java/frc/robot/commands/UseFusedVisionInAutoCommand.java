// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: UseFusedVisionForPositionInAutoCommand.java
// Intent: Forms to make us use vision for odometry updates during auto
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.control.Constants;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * Forms a command to use fused vision
 * uses it for 15seconds (the auto duration)
 */
public class UseFusedVisionInAutoCommand extends Command
{
  private DrivetrainSubsystem drivetrainSubsystem;
  private Timer timer = new Timer();
  private boolean done = false;
  
  /** 
  * creates a new fused vison in auto command
  * @param drivetrainSubsystem - the drivetrain subsystem
  */
  public UseFusedVisionInAutoCommand(DrivetrainSubsystem drivetrainSubsystem)
  {
    this.drivetrainSubsystem = drivetrainSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    timer.reset();
    timer.start();
    done = false;
    DataLogManager.log("Starting UseFusedVisionInAutoCommand");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    drivetrainSubsystem.setUseVision(true);
    if (timer.hasElapsed(Constants.autoUseFusedVisionDuration))
    {
      drivetrainSubsystem.setUseVision(false);
      done = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    drivetrainSubsystem.setUseVision(false);
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