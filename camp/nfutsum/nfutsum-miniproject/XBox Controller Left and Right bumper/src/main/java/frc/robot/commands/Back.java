// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Motor;
import edu.wpi.first.wpilibj2.command.CommandBase;


/** An example command that uses an example subsystem. */
public class Back extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final Motor m_subsystem;

  /**
   * Constructer that takes the motor subsystem
   *
   * @param m_motor The motor subsystem
   */
  public Back(Motor m_motor) {
    m_subsystem = m_motor;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_motor);
  
  }



// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Runs motor backward whenever command is called
  @Override
  public void execute() {
    this.m_subsystem.backward();
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //need to stop the motor here.
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
