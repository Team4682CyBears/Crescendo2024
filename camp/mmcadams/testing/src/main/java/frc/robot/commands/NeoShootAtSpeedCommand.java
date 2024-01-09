package frc.robot.commands;

import frc.robot.subsystems.NeoShooterSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class NeoShootAtSpeedCommand extends CommandBase {

  private NeoShooterSubsystem shooterSubsystem;
  private double leftRpm = 2000;
  private double rightRpm = 1500;
  private boolean isDone = false;

  public NeoShootAtSpeedCommand(NeoShooterSubsystem theShooterSubsystem) {
    shooterSubsystem = theShooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isDone = false;
    System.out.println("init of NeoShootAtSpeedCommand ... ");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.shooterSubsystem.setShooterVelocityLeft(leftRpm, leftRpm);
    this.shooterSubsystem.setShooterVelocityRight(rightRpm, rightRpm);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isDone = true;
    this.shooterSubsystem.setAllStop();
    System.out.println("end of NeoShootAtSpeedCommand ... ");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
