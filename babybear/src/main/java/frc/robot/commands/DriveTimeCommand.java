// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Rapid React - 2023
// File: DriveTimeCommand.java
// Intent: Forms a command to drive the wheels according to input parameters (encoder dead reckoning and accelormeter for rotation).
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainSubsystem;

public class DriveTimeCommand extends CommandBase
{
  private DrivetrainSubsystem drivetrain;
  private Timer timer = new Timer();
  private boolean done = false;
  private double xVelocity = 0.0;
  private double yVelocity = 0.0;
  private double rotVelocity = 0.0;
  private double durationSecondsValue = 0.0;
  
  /** 
  * Creates a new driveCommand. 
  * 
  * @param drivetrainSubsystem - the drive train subsystem
  * @param x - the x velocity
  * @param y - the y velocity
  */
  public DriveTimeCommand(
    DrivetrainSubsystem drivetrainSubsystem,
    double x,
    double y,
    double rot,
    double durationSeconds)
  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drivetrain = drivetrainSubsystem;
    addRequirements(drivetrainSubsystem);

    xVelocity = x;
    yVelocity = y;
    rotVelocity = rot;
    durationSecondsValue = durationSeconds;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize()
  {
    drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    timer.reset();
    timer.start();
    done = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute()
  {
    drivetrain.drive(
      new ChassisSpeeds(xVelocity, yVelocity, rotVelocity));
    if (timer.hasElapsed(this.durationSecondsValue))
    {
      done = true;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted)
  {
    drivetrain.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
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
}
