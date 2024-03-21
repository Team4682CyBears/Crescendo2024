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
import frc.robot.subsystems.ShooterAngleSubsystem;
import frc.robot.subsystems.ShooterOutfeedSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.function.DoubleSupplier; 

/**
 * Forms a command to shoot the shooter
 */
public class ShooterShootCommand extends Command {

  private ShooterOutfeedSubsystem shooterOutfeed;
  private ShooterAngleSubsystem shooterAngle;
  private FeederSubsystem feeder;
  private double desiredAngleDegrees; 
  private double desiredSpeedRpm; 
  private DoubleSupplier desiredSpeedRpmSupplier; 
  private boolean setSpeedsFromSupplier = false;
  private boolean isAtDesiredAngle = false;
  private boolean isDone = false;
  private Timer timer = new Timer();
  private Timer delayTimer = new Timer();
  private boolean isAtDesiredAngleBaseline = false;

  /**
   * Constructor for ShooterShootCommand
   * Will set shooter to desired angle and speeds before shooting
   * @param desiredAngleDegrees
   * @param desiredSpeedRpm
   * @param shooterOutfeed
   * @param shooterAngle
   * @param feeder
   */
  public ShooterShootCommand(
      double desiredAngleDegrees,
      double desiredSpeedRpm,
      ShooterOutfeedSubsystem shooterOutfeed,
      ShooterAngleSubsystem shooterAngle,
      FeederSubsystem feeder) {
    this.desiredAngleDegrees = desiredAngleDegrees;
    this.desiredSpeedRpm = desiredSpeedRpm;
    this.isAtDesiredAngle = false;
    this.isAtDesiredAngleBaseline = this.isAtDesiredAngle;
    this.shooterOutfeed = shooterOutfeed;
    this.shooterAngle = shooterAngle;
    this.feeder = feeder;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterOutfeed, shooterAngle, feeder);
  }

  /**
   * Constructor for ShooterShootCommand
   * Will set shooter to desired angle before shooting
   * uses default shooter speeds
   * 
   * @param desiredAngleDegrees
   * @param shooterOutfeed
   * @param shooterAngle
   * @param feeder
   */
  public ShooterShootCommand(double desiredAngleDegrees, ShooterOutfeedSubsystem shooterOutfeed,
      ShooterAngleSubsystem shooterAngle, FeederSubsystem feeder) {
    this(desiredAngleDegrees, Constants.shooterDefaultSpeedRpm, 
    shooterOutfeed, shooterAngle, feeder);
  }

  /**
   * Constructor for ShooterShootCommand
   * assumes shooter is already at the desired angle
   * uses specified speeds
   * @param desiredSpeedSpeedRpm
   * @param shooterOutfeed
   * @param feeder
   */
  public ShooterShootCommand(double desiredSpeedRpm,
  ShooterOutfeedSubsystem shooterOutfeed, FeederSubsystem feeder) {
    this.desiredSpeedRpm = desiredSpeedRpm;
    this.isAtDesiredAngle = true;
    this.isAtDesiredAngleBaseline = this.isAtDesiredAngle;
    this.shooterOutfeed = shooterOutfeed;
    this.feeder = feeder;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterOutfeed, feeder);
  }

  /**
   * Constructor for ShooterShootCommand
   * assumes shooter is already at the desired angle
   * uses specified speed suppliers
   * @param desiredSpeedRpmSupplier
   * @param shooterOutfeed
   * @param feeder
   */
  public ShooterShootCommand(DoubleSupplier desiredSpeedRpmSupplier,
  ShooterOutfeedSubsystem shooterOutfeed, FeederSubsystem feeder) {
    this.desiredSpeedRpmSupplier = desiredSpeedRpmSupplier;
    this.setSpeedsFromSupplier = true;
    this.isAtDesiredAngle = true;
    this.isAtDesiredAngleBaseline = this.isAtDesiredAngle;
    this.shooterOutfeed = shooterOutfeed;
    this.feeder = feeder;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(shooterOutfeed, feeder);
  }

  /**
   * Constructor for ShooterShootCommand
   * assumes shooter is already at the desired angle
   * uses default shooter speeds
   * @param shooterOutfeed
   * @param feeder
   */
  public ShooterShootCommand(ShooterOutfeedSubsystem shooterOutfeed, FeederSubsystem feeder) {
    this(Constants.shooterDefaultSpeedRpm, shooterOutfeed, feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.isAtDesiredAngle = this.isAtDesiredAngleBaseline;
    if (!isAtDesiredAngle) {
      System.out.println("Setting Angle to " + desiredAngleDegrees + " degrees");
    }

    if (setSpeedsFromSupplier) {
      this.desiredSpeedRpm = this.desiredSpeedRpmSupplier.getAsDouble();
    }

    // stop the feeder so the note doesn't go through shooter before shooter is setup
    feeder.setAllStop();
    feeder.setFeederMode(FeederMode.FeedToShooter);
    // set shooter angle and speeds
    if (!isAtDesiredAngle) {
      shooterAngle.setAngleDegrees(desiredAngleDegrees);
    }

    System.out.println("Spinning up shooter...");
    System.out.println("Target RPM: " + desiredSpeedRpm);
    shooterOutfeed.setShooterVelocity(desiredSpeedRpm);

    timer.reset();
    delayTimer.reset();
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // wait for shooter to be a the right angle and spped
    if (!isAtDesiredAngle){
      isAtDesiredAngle = shooterAngle.isAngleWithinTolerance(desiredAngleDegrees);
    }
    if (isAtDesiredAngle && shooterOutfeed.isAtSpeed()){
      System.out.println("Shooter at desired speed and angle.");
      System.out.println("Wait for feeder delay...");
      delayTimer.start();
    }
    if (delayTimer.hasElapsed(Constants.shooterSpinUpDelay)){
      feeder.setFeederSpeed(Constants.feederSpeed);
      System.out.println("Feeding Note...");
      timer.start();
    }
    if (timer.hasElapsed(Constants.shooterShootDuration)){
      feeder.setAllStop();
      shooterOutfeed.setAllStop();
      isDone = true;
    }
    else {
      shooterOutfeed.setShooterVelocity(desiredSpeedRpm);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    feeder.setAllStop();
    shooterOutfeed.setAllStop();
    isDone = true;
    System.out.println("end of ShootAtSpeedCommand ... ");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }

}