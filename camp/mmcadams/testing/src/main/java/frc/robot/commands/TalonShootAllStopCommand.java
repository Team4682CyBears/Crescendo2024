package frc.robot.commands;

import frc.robot.subsystems.TalonShooterSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TalonShootAllStopCommand extends CommandBase {
  
  private final TalonShooterSubsystem shooterSubsystem;

  public TalonShootAllStopCommand(TalonShooterSubsystem theShooterSubsystem) {
    shooterSubsystem = theShooterSubsystem;
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterSubsystem.setAllStop();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
