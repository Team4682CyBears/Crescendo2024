// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: MotorBackwardCommand.java
// Intent: Creates a command to move the motor backward
// ************************************************************

package frc.robot.commands;

import frc.robot.subsystems.MotorSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;


/** An example command that uses an example subsystem. */
public class MotorBackwardCommand extends CommandBase {
  private final MotorSubsystem theSubsystem;

  /**
   * Constructer that takes the motor subsystem
   *
   * @param subsystem The motor subsystem
   */
  public MotorBackwardCommand(MotorSubsystem subsystem) {
    theSubsystem = subsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(subsystem);
  }

  // Runs motor backward whenever command is called
  @Override
  public void execute() {
    this.theSubsystem.setBackward();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //need to stop the motor here.
    this.theSubsystem.setStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
