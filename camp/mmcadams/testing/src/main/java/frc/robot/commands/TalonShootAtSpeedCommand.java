package frc.robot.commands;

import frc.robot.subsystems.TalonShooterSubsystem;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TalonShootAtSpeedCommand extends CommandBase {

  private TalonShooterSubsystem shooterSubsystem;
  private double baseRpm = 6500;
  private double leftRpm = baseRpm * 1.0;
  private double rightRpm = baseRpm * 0.5;
  private boolean isDone = false;

  public TalonShootAtSpeedCommand(TalonShooterSubsystem theShooterSubsystem) {
    shooterSubsystem = theShooterSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    isDone = false;
    System.out.println("init of TalonShootAtSpeedCommand ... ");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.shooterSubsystem.setShooterVelocityLeft(leftRpm);
    this.shooterSubsystem.setShooterVelocityRight(rightRpm);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    isDone = true;
    System.out.println("end of TalonShootAtSpeedCommand ... ");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
