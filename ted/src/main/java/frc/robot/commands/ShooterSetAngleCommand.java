// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: ShooterSetAngleCommand.java
// Intent: Forms a command to set the shooter angle
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
public class ShooterSetAngleCommand extends Command {

  private TalonShooterSubsystem shooter;
  private double desiredAngleDegrees; 

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
    addRequirements(shooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.setAngleDegrees(desiredAngleDegrees);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("end of ShootAtSpeedCommand ... ");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true; 
  }

}