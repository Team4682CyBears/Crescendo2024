package frc.robot.commands;

import frc.robot.subsystems.TalonShooterSubsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class TalonShootAtSpeedCommand extends CommandBase {

  private TalonShooterSubsystem shooterSubsystem;
  private double baseRpm = 6500;
  private double leftSpeedDefault = 1.0;
  private double rightSpeedDefault = 0.5;
  private double leftRpm = leftSpeedDefault * baseRpm;
  private double rightRpm = rightSpeedDefault * baseRpm;
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
    // get speeds from SmartDashboard. Valid values from [0.0, 1.0]
    double leftSpeed = SmartDashboard.getNumber("ShooterLeftSpeed", leftSpeedDefault);
    double rightSpeed = SmartDashboard.getNumber("ShooterRightSpeedDefault", rightSpeedDefault);
    leftRpm = leftSpeed * baseRpm;
    rightRpm = rightSpeed * baseRpm; 
    System.out.println("Setting left motor speed to " + leftSpeed);
    System.out.println("Setting right motor speed to " + rightSpeed);
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
