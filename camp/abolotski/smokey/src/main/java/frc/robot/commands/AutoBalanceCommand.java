package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.common.MotorUtils;
import frc.robot.common.VectorUtils;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

import java.lang.Math;

/**
 * Implements a command to perform a auto balancing routine. 
 */
public class AutoBalanceCommand extends CommandBase{
  private double velocityValue = 0.3;
  private PIDController pidController = new PIDController(velocityValue/Math.sin(Math.toRadians(10)),0.0,0.0);

  private DrivetrainSubsystem drivetrainsubsystem = null;

  /**
   * Constructor for auto balance command.
   */
  public AutoBalanceCommand(DrivetrainSubsystem drivetrainSubsystem) {
    this.drivetrainsubsystem = drivetrainSubsystem;

    // do not need to add Navx as a requirement because it is read-only
    addRequirements(drivetrainSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    drivetrainsubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  { 
    // Not sure if it's too computationally expensive to read the NavX on every time tick.  
    // If this costs too much, could read on every Nth time through the loop   
    Translation2d angleOfSteepestAscent = VectorUtils.getAngleOfSteepestAscent(this.drivetrainsubsystem.getEulerAngle());
    Translation2d velocityVec = normalizeXYVelocities(angleOfSteepestAscent);
    drivetrainsubsystem.drive(new ChassisSpeeds(velocityVec.getX(), velocityVec.getY(), 0.0d));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    drivetrainsubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished()
  {
    return this.drivetrainsubsystem.isLevel();
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
    return new Translation2d((angles.getX()/h) * velocity, (angles.getY()/h) * velocity);
  }

}
