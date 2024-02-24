// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: ShooterShootCommand.java
// Intent: Forms a command to shoot the shooter.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import frc.robot.common.FeederMode;
import frc.robot.control.Constants;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.TalonShooterSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Forms a command to shoot the shooter
 */
public class ShooterShootCommand extends Command {

  private TalonShooterSubsystem shooter;
  private FeederSubsystem feeder;
  private double desiredAngleDegrees; 
  private double desiredLeftSpeedRpm; 
  private double desiredRightSpeedRpm;  
  private boolean isAtDesiredAngle = false;
  private boolean isDone = false;
  private Timer timer = new Timer();

  /**
   * Constructor for ShooterShootCommand
   * Will set shooter to desired angle and speeds before shooting
   * @param desiredAngleDegrees
   * @param desiredLeftSpeedRpm
   * @param desiredRightSpeedRpm
   * @param shooter
   * @param feeder
   */
  public ShooterShootCommand(
      double desiredAngleDegrees,
      double desiredLeftSpeedRpm,
      double desiredRightSpeedRpm,
      TalonShooterSubsystem shooter,
      FeederSubsystem feeder) {
    this.desiredAngleDegrees = desiredAngleDegrees;
    this.desiredLeftSpeedRpm = desiredLeftSpeedRpm;
    this.desiredRightSpeedRpm = desiredRightSpeedRpm;
    this.isAtDesiredAngle = false;
    this.shooter = shooter;
    this.feeder = feeder;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, feeder);
  }

  /**
   * Constructor for ShooterShootCommand
   * Will set shooter to desired angle before shooting
   * uses default shooter speeds
   * 
   * @param desiredAngleDegrees
   * @param shooter
   * @param feeder
   */
  public ShooterShootCommand(double desiredAngleDegrees, TalonShooterSubsystem shooter,
      FeederSubsystem feeder) {
    this(desiredAngleDegrees, Constants.shooterLeftDefaultSpeedRpm, Constants.shooterRightDefaultSpeedRpm, 
    shooter, feeder);
  }

  /**
   * Constructor for ShooterShootCommand
   * assumes shooter is already at the desired angle
   * uses specified speeds
   * @param desiredLeftSpeedRpm
   * @param desiredRightSpeedRpm
   * @param shooter
   * @param feeder
   */
  public ShooterShootCommand(double desiredLeftSpeedRpm, double desiredRightSpeedRpm,
  TalonShooterSubsystem shooter, FeederSubsystem feeder) {
    this.desiredLeftSpeedRpm = desiredLeftSpeedRpm;
    this.desiredRightSpeedRpm = desiredRightSpeedRpm;
    this.isAtDesiredAngle = true;
    this.shooter = shooter;
    this.feeder = feeder;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooter, feeder);
  }

  /**
   * Constructor for ShooterShootCommand
   * assumes shooter is already at the desired angle
   * uses default shooter speeds
   * @param shooter
   * @param feeder
   */
  public ShooterShootCommand(TalonShooterSubsystem shooter, FeederSubsystem feeder) {
    this(Constants.shooterLeftDefaultSpeedRpm, Constants.shooterRightDefaultSpeedRpm, shooter, feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // stop the feeder so the note doesn't go through shooter before shooter is setup
    feeder.setAllStop();
    feeder.setFeederMode(FeederMode.FeedToShooter);
    // set shooter angle and speeds
    if (!isAtDesiredAngle) {
      shooter.setAngleDegrees(desiredAngleDegrees);
    }
    shooter.setShooterVelocityLeft(desiredLeftSpeedRpm);
    shooter.setShooterVelocityRight(desiredRightSpeedRpm);

    timer.reset();
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // wait for shooter to be a the right angle and spped
    if (!isAtDesiredAngle){
      isAtDesiredAngle = shooter.isAngleWithinTolerance(desiredAngleDegrees);
    }
    if (isAtDesiredAngle && shooter.isAtSpeed()){
      feeder.setFeederSpeed(Constants.feederSpeed);
      timer.start();
    }
    if (timer.hasElapsed(Constants.shooterShootDuration)){
      feeder.setAllStop();
      shooter.setAllStop();
      isDone = true;
    }
    else {
      shooter.setShooterVelocityLeft(desiredLeftSpeedRpm);
      shooter.setShooterVelocityRight(desiredRightSpeedRpm);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.setAllStop();
    shooter.setAllStop();
    isDone = true;
    System.out.println("end of ShootAtSpeedCommand ... ");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }

}