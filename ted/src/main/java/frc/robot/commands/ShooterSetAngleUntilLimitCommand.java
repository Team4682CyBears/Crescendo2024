// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: ShooterSetAngleUntilLimitCommand.java
// Intent: Forms a command to update the shooter angle in one direction until the directional limit is reached
// or until the command scheduler stops its execution (whichever comes first)
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import frc.robot.control.Constants;
import frc.robot.subsystems.ShooterAngleSubsystem;
import edu.wpi.first.wpilibj2.command.Command; 

/**
 * Forms a command to update the shooter angle in one direction until the directional limit is reached
 * or until the command scheduler stops its execution (whichever comes first)
 */
public class ShooterSetAngleUntilLimitCommand extends Command {

  private ShooterAngleSubsystem shooter;
  private boolean increasingAngle = false;
  private boolean done = false;

  /**
   * Constructor for ShooterSetAngleTesterCommand
   * Will update the shooter angle in one direction until the directional limit is reached (maximum limit for the 
   * increasing direction and minimum limit for decreasing) or until the command is interrupted by the scheduler 
   * @param increaseAngle - When true the shooter angle will increase until either the maximum limit is reached or 
   * @param shooter - the shooter angle subsystem in use currently
   */
  public ShooterSetAngleUntilLimitCommand(boolean increaseAngle, ShooterAngleSubsystem shooter) {
    this.increasingAngle = increaseAngle;
    this.shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    done = false;
  }

  // buisness logic to move the shooter once per robot tick (every 20ms / 50Hz)
  @Override
  public void execute() {

    double angleStepSize = Constants.shooterAngleStickIncrementMagnitude;
    double currentShooterAngle = this.shooter.getAngleDegrees();
    double proposedAngle = currentShooterAngle;

    if(increasingAngle ) {
        if(currentShooterAngle >= Constants.shooterAngleMaxDegrees) {
            done = true;
        }
        else {
            proposedAngle = currentShooterAngle + angleStepSize;
        }
    }
    else {
        if(currentShooterAngle <= Constants.shooterAngleMinDegrees) {
            done = true;
        }
        else {
            proposedAngle = currentShooterAngle - angleStepSize;
        }
    }

    if(!done) {
        this.shooter.setAngleDegrees(proposedAngle);
    }
  }
 
  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(!done){
      // this will have the effect of stopping the angle movement
      // at the current angle
      this.shooter.setAngleDegrees(this.shooter.getAngleDegrees());
    }
    if(interrupted){
      done = true;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }

}