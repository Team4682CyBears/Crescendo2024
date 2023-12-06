// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.MotorSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;


/** An example command that uses an example subsystem. */
public class MotorBackwardCommand extends CommandBase {
  private final MotorSubsystem theSubsystem;

  /**
   * Constructer that takes the motor subsystem
   *
   * @param m_motor The motor subsystem
   */
  public MotorBackwardCommand(MotorSubsystem m_motor) {
    theSubsystem = m_motor;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_motor);
  
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
