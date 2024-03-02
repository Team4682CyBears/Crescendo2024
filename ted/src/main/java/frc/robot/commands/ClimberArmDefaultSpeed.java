// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: ClimberArmDefaultSpeed.java
// Intent: Forms a command to set a climber motors using speeds
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import frc.robot.common.MotorUtils;
import frc.robot.control.Constants;
import frc.robot.control.ManualInputInterfaces;
import frc.robot.subsystems.ClimberSubsystem;
import java.util.function.DoubleSupplier;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Forms a command to have the climbers climb or retract
 */
public class ClimberArmDefaultSpeed extends Command {

  private ClimberSubsystem climber;
  private DoubleSupplier leftSpeed;
  private DoubleSupplier rightSpeed;

  /**
   * Constructor for ShooterShootCommand
   * Will set shooter to desired angle before shooting
   * @param climberSubsystem - the target climber subsystem
   * @param leftSpeedDoubleSupplier - the left climber motor speed supplier
   * @param rightSpeedDoubleSupplier - the left climber motor speed supplier
   */
  public ClimberArmDefaultSpeed(
    ClimberSubsystem climberSubsystem,
    DoubleSupplier leftSpeedDoubleSupplier,
    DoubleSupplier rightSpeedDoubleSupplier ) {
    this.climber = climberSubsystem;
    this.leftSpeed = leftSpeedDoubleSupplier;
    this.rightSpeed = rightSpeedDoubleSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
    this.addRequirements(this.climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // with this being the default operation we need to make sure that the mode in effect allows us to do the operation
    this.climber.setClimberSpeeds(this.leftSpeed.getAsDouble(), this.rightSpeed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true; 
  } 

}