package frc.robot.commands;

import frc.robot.subsystems.MotorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

public class SpeedUp extends Command{
    private final MotorSubsystem motor_subsystem;

    public SpeedUp(MotorSubsystem subsystem) {
    motor_subsystem = subsystem;
    addRequirements(subsystem);
  }

  @Override
  public void initialize() {
    
  }

  @Override
  public void execute() {

  }

  @Override
  public void end(boolean interrupted) {

  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
