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
import frc.robot.subsystems.ShooterAngleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Forms a command to shoot the shooter
 */
public class ShooterSetAngleCommand extends Command {

  private ShooterAngleSubsystem shooterAngleSubsystem;
  protected double desiredAngleDegrees; 
  protected boolean done = false;

  /**
   * Constructor for ShooterShootCommand
   * Will set shooter to desired angle before shooting
   * @param desiredAngleDegrees
   * @param shooterAngle
   */
  public ShooterSetAngleCommand(double desiredAngleDegrees, ShooterAngleSubsystem shooterAngleSubsystem) {
    this.desiredAngleDegrees = desiredAngleDegrees;
    this.shooterAngleSubsystem = shooterAngleSubsystem;
    // Use addRequirements() here to declare subsystem dependencies.
    this.addRequirements(shooterAngleSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.done = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooterAngleSubsystem.setAngleDegrees(desiredAngleDegrees); 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted){
      done = true;
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooterAngleSubsystem.isAngleWithinTolerance(desiredAngleDegrees);
  }

}