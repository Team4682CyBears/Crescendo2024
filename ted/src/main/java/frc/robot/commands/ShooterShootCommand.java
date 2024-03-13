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
  private double desiredLeftSpeedRpm; 
  private double desiredRightSpeedRpm;  
  private DoubleSupplier desiredLeftSpeedRpmSupplier; 
  private DoubleSupplier desiredRightSpeedRpmSupplier;  
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
   * @param desiredLeftSpeedRpm
   * @param desiredRightSpeedRpm
   * @param shooterOutfeed
   * @param shooterAngle
   * @param feeder
   */
  public ShooterShootCommand(
      double desiredAngleDegrees,
      double desiredLeftSpeedRpm,
      double desiredRightSpeedRpm,
      ShooterOutfeedSubsystem shooterOutfeed,
      ShooterAngleSubsystem shooterAngle,
      FeederSubsystem feeder) {
    this.desiredAngleDegrees = desiredAngleDegrees;
    this.desiredLeftSpeedRpm = desiredLeftSpeedRpm;
    this.desiredRightSpeedRpm = desiredRightSpeedRpm;
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
    this(desiredAngleDegrees, Constants.shooterLeftDefaultSpeedRpm, Constants.shooterRightDefaultSpeedRpm, 
    shooterOutfeed, shooterAngle, feeder);
  }

  /**
   * Constructor for ShooterShootCommand
   * assumes shooter is already at the desired angle
   * uses specified speeds
   * @param desiredLeftSpeedRpm
   * @param desiredRightSpeedRpm
   * @param shooterOutfeed
   * @param feeder
   */
  public ShooterShootCommand(double desiredLeftSpeedRpm, double desiredRightSpeedRpm,
  ShooterOutfeedSubsystem shooterOutfeed, FeederSubsystem feeder) {
    this.desiredLeftSpeedRpm = desiredLeftSpeedRpm;
    this.desiredRightSpeedRpm = desiredRightSpeedRpm;
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
   * @param desiredLeftSpeedRpmSupplier
   * @param desiredRightSpeedRpmSupplier
   * @param shooterOutfeed
   * @param feeder
   */
  public ShooterShootCommand(DoubleSupplier desiredLeftSpeedRpmSupplier, DoubleSupplier desiredRightSpeedRpmSupplier,
  ShooterOutfeedSubsystem shooterOutfeed, FeederSubsystem feeder) {
    this.desiredLeftSpeedRpmSupplier = desiredLeftSpeedRpmSupplier;
    this.desiredRightSpeedRpmSupplier = desiredRightSpeedRpmSupplier;
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
    this(Constants.shooterLeftDefaultSpeedRpm, Constants.shooterRightDefaultSpeedRpm, shooterOutfeed, feeder);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.isAtDesiredAngle = this.isAtDesiredAngleBaseline;
    if (!isAtDesiredAngle) {
      System.out.printf("Setting Angle to %.2f degrees \n", desiredAngleDegrees);
    }

    if (setSpeedsFromSupplier) {
      this.desiredLeftSpeedRpm = this.desiredLeftSpeedRpmSupplier.getAsDouble();
      this.desiredRightSpeedRpm = this.desiredRightSpeedRpmSupplier.getAsDouble();
    }

    // stop the feeder so the note doesn't go through shooter before shooter is setup
    feeder.setAllStop();
    feeder.setFeederMode(FeederMode.FeedToShooter);
    // set shooter angle and speeds
    if (!isAtDesiredAngle) {
      shooterAngle.setAngleDegrees(desiredAngleDegrees);
    }

    System.out.println("Spinning up shooter...");
    System.out.printf("Target RPM: Left %1$.0f. Right RPM: %2$.0f \n", desiredLeftSpeedRpm, desiredRightSpeedRpm);
    shooterOutfeed.setShooterVelocityLeft(desiredLeftSpeedRpm);
    shooterOutfeed.setShooterVelocityRight(desiredRightSpeedRpm);

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
    if (isAtDesiredAngle && shooterOutfeed.isAtSpeed(desiredLeftSpeedRpm, desiredRightSpeedRpm)){
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
      shooterOutfeed.setShooterVelocityLeft(desiredLeftSpeedRpm);
      shooterOutfeed.setShooterVelocityRight(desiredRightSpeedRpm);
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