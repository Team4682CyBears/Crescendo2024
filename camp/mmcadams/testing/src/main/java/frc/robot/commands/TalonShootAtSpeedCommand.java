package frc.robot.commands;

import frc.robot.subsystems.TalonShooterSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TalonShootAtSpeedCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final TalonShooterSubsystem shooterSubsystem;

  public TalonShootAtSpeedCommand(TalonShooterSubsystem theShooterSubsystem) {
    shooterSubsystem = theShooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println("setting velocity ... ");
    this.shooterSubsystem.setShooterVelocityRight(1500);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("end just happened!!");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
