package frc.robot.commands;

import frc.robot.subsystems.NeoShooterSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class NeoShootAllStopCommand extends CommandBase {
  
  private final NeoShooterSubsystem shooterSubsystem;

  public NeoShootAllStopCommand(NeoShooterSubsystem theShooterSubsystem) {
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
