// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up 2023
// File: UseVisionForPositionCommand.java
// Intent: Forms a command to use vision for position updates
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * Implements a command
 */
public class UseVisionForPositionCommand extends CommandBase{

  private boolean done = false;

  private DrivetrainSubsystem drivetrainsubsystem = null;
  private Timer commandTimer = new Timer();
  private double commandDurationSeconds = 0.5;

  /**
   * Constructor for command.
   */
  public UseVisionForPositionCommand (DrivetrainSubsystem drivetrainSubsystem) {
    this.drivetrainsubsystem = drivetrainSubsystem;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    commandTimer.reset();
    commandTimer.start();
    this.drivetrainsubsystem.setUseVision(true);
    done = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute(){ 
    if (commandTimer.hasElapsed(this.commandDurationSeconds))
    {
      this.drivetrainsubsystem.setUseVision(false);
      done = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted){
    this.drivetrainsubsystem.setUseVision(false);
    done = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished(){
    return done;
  }

}
