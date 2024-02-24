// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: ShooterSetAngleCommand.java
// Intent: Forms a command to set the shooter angle
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import frc.robot.control.Constants;
import frc.robot.subsystems.TalonShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Forms a command to shoot the shooter
 */
public class ShooterSetAngleCommand extends Command {

  private TalonShooterSubsystem shooter;
  protected double desiredAngleDegrees; 
  private boolean updatedAngleSet = false;
  private boolean done = false;

  /**
   * Constructor for ShooterShootCommand
   * Will set shooter to desired angle before shooting
   * @param desiredAngleDegrees
   * @param shooter
   */
  public ShooterSetAngleCommand(double desiredAngleDegrees, TalonShooterSubsystem shooter) {
    this.desiredAngleDegrees = desiredAngleDegrees;
    this.shooter = shooter;
    // Use addRequirements() here to declare subsystem dependencies.
    this.addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.updatedAngleSet = false;
    this.done = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!this.updatedAngleSet) {
      shooter.setAngleDegrees(desiredAngleDegrees);
    }
    else {
      done = shooter.isAngleWithinTolerance(Constants.shooterAngleToleranceDegrees);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted){
      done = true;
      System.out.println("interrupted end of ShooterSetAngleCommand ... ");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done; 
  }

}