package frc.robot.commands;

import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class ShootAtSpeedCommand extends Command {

  private ShooterSubsystem shooterSubsystem;
  private double baseRpm = 2500;
  private double leftRpm = baseRpm * 1.0;
  private double rightRpm = baseRpm * 0.5;
  private boolean isDone = false;

  public ShootAtSpeedCommand(ShooterSubsystem theShooterSubsystem) {
    shooterSubsystem = theShooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isDone = false;
    System.out.println("init of ShootAtSpeedCommand ... ");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.shooterSubsystem.setShooterVelocityLeft(leftRpm);
//    this.shooterSubsystem.setShooterVelocityRight(rightRpm);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isDone = true;
    System.out.println("end of ShootAtSpeedCommand ... ");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}