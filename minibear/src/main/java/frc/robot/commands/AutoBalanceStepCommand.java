package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.common.MotorUtils;
import frc.robot.common.VectorUtils;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

import java.lang.Math;

/**
 * Implements a command that will repeatedly iterate small drive steps and measurements to 
 * determine if the robot has obtained level, therefore attaining auto balance on the ramp.
 */
public class AutoBalanceStepCommand extends CommandBase{
  private Timer driveTimer = new Timer();
  private Timer waitTimer = new Timer();
  private boolean done = false;
  private double xVelocity = 0.0;
  private double yVelocity = 0.0;
  private double rotVelocity = 0.0;
  // setting wait duration to 0 implements a continuous motion
  private double waitDurationSecondsValue = 0.0;
  private double driveDurationSecondsValue = 0.15;  
  // higher velocity values caused the robot to slip on the polycarb and not advance as far
  private double velocityValue = 0.36;
  private int numIterations = 0;
  // ramp max slope = 15 degrees, and then typically <10 degrees once the robot is mostly on.  
  // use velocity value for errors >= sin(10) degrees to have high velocity for getting onto the ramp
  // and then proprortaional for smaller errors to get the fine control needed to balance.  
  private PIDController pidController = new PIDController(velocityValue/Math.sin(Math.toRadians(10)),0.001,0.0);

  private DrivetrainSubsystem drivetrainsubsystem = null;

  /**
   * A constructor for auto balance step command
   * @param drivetrainSubsystem
   * @param navxsubsystem
   */
  public AutoBalanceStepCommand(DrivetrainSubsystem drivetrainSubsystem) {
    this.drivetrainsubsystem = drivetrainSubsystem;

    // do not need to add Navx as a requirement because it is read-only
    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Translation2d angleOfSteepestAscent = VectorUtils.getAngleOfSteepestAscent(this.drivetrainsubsystem.getEulerAngle());
    Translation2d velocityVec = normalizeXYVelocities(angleOfSteepestAscent);
    xVelocity = velocityVec.getX();
    yVelocity = velocityVec.getY();

    drivetrainsubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    driveTimer.reset();
    waitTimer.reset();
    driveTimer.start();
    done = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {

    // drive time interval followed by wait time interval
    if (driveTimer.hasElapsed(this.driveDurationSecondsValue))
    {
      drivetrainsubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
      waitTimer.start();
    }
    // drive along the vector of steepest ascent
    else {
      drivetrainsubsystem.drive(new ChassisSpeeds(xVelocity, yVelocity, rotVelocity));
    }

    if (waitTimer.hasElapsed(this.waitDurationSecondsValue))
    {
      // test for level at the end of the wait cycle. 
      // TODO could also stop after maxItermations here with || (numIterations >= maxIterations))
      if (drivetrainsubsystem.isLevel()){
        System.out.println("Ramp is Level. Completing Auto Balance Step Command.");
        done = true;
      } else {
        // setup the next drive cycle
        numIterations += 1;
        System.out.println("auto balance step command: completed cycle " + numIterations + ".");
        System.out.println("RecentPitches " + this.drivetrainsubsystem.getRecentPitches());
        System.out.println("RecentRolls " + this.drivetrainsubsystem.getRecentRolls());
        Translation2d angleOfSteepestAscent = VectorUtils.getAngleOfSteepestAscent(this.drivetrainsubsystem.getEulerAngle());
        Translation2d velocityVec = normalizeXYVelocities(angleOfSteepestAscent);
        xVelocity = velocityVec.getX();
        yVelocity = velocityVec.getY();

        driveTimer.reset();
        waitTimer.reset();
        driveTimer.start();
      }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    drivetrainsubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    if(interrupted)
    {
      done = true;      
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return done;
  }

  /**
   * Scales XY values to the desired velocity 
   * @param angles - Translation2d vector representing angle of steepest ascent
   * @return - Translation2d scaled vector
   */
  private Translation2d normalizeXYVelocities(Translation2d angles)
  {
    double h = Math.sqrt(Math.pow(angles.getX(), 2) + Math.pow(angles.getY(), 2));
    double velocity = 0;
    // h is always positive, so the pidController will always return a negative. 
    // Take the absolute value of the output of pidController to always have positive velcoty.
    // Directionality is already accounted for in X and Y.    
    velocity = Math.abs(pidController.calculate(h, 0.0));
    velocity = MotorUtils.clamp(velocity, 0.0, velocityValue);
    System.out.println("Setting velocity to " + velocity + " for angle error " + h);
    
    return new Translation2d((angles.getX()/h) * velocity, (angles.getY()/h) * velocity);
  }

}
