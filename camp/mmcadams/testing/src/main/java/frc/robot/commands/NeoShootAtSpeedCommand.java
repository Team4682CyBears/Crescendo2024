package frc.robot.commands;

import frc.robot.subsystems.NeoShooterSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class NeoShootAtSpeedCommand extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  private final NeoShooterSubsystem shooterSubsystem;
  private double leftRpm = 2000;
  private double rightRpm = 1500;

  public NeoShootAtSpeedCommand(NeoShooterSubsystem theShooterSubsystem) {
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
    this.shooterSubsystem.setShooterVelocityLeft(leftRpm, leftRpm);
    this.shooterSubsystem.setShooterVelocityRight(rightRpm, rightRpm);
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
