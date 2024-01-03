// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: MotorStopCommand.java
// Intent: Creates a command to stop the motor
// ************************************************************

package frc.robot.commands;

import frc.robot.subsystems.MotorSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** 
 * MotorStopCommand class 
 */
public class MotorStopCommand extends CommandBase {
  private final MotorSubsystem theSubsystem;

  /**
   * Constructer that takes in the motor subsystem
   *
   * @param subsystem motor subsystem
   */
  public MotorStopCommand(MotorSubsystem subsystem) {
    theSubsystem = subsystem;
    addRequirements(subsystem);
  }

  /** 
   * Stops motor movement when command is called
  */
  @Override
  public void execute() {
    this.theSubsystem.setStop();
  }

  /** 
   *Returns true when the command should end.
   */ 
  @Override
  public boolean isFinished() {
    return true;
  }
}
