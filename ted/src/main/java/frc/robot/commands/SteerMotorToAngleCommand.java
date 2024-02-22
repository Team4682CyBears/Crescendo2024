// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: SteerMotorToAngleCommand.java
// Intent: Forms a command to set a steer motor to a test spot / angle.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import frc.robot.subsystems.SteerMotorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class SteerMotorToAngleCommand extends Command {

  private SteerMotorSubsystem steerMotorSubsystem;
  private boolean isDone = false;
  private double targetDegrees = 0.0;

  public SteerMotorToAngleCommand(SteerMotorSubsystem steerSubsystem, double degrees) {
    steerMotorSubsystem = steerSubsystem;
    targetDegrees = degrees;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(steerMotorSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isDone = false;
    steerMotorSubsystem.setSteerMotorAngle(targetDegrees);
    System.out.println("init of SteerMotorToAngleCommand ... ");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!isDone) {
        isDone = steerMotorSubsystem.isAtTargetPosition();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isDone = true;
    System.out.println("end of SteerMotorToAngleCommand ... ");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}