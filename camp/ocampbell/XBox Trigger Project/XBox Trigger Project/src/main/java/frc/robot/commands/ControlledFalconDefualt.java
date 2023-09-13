// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// File: ControlledFalconDEfualt.java
// Intent: Assigns a motor speed value to a falcon.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 
package frc.robot.commands;

import frc.robot.control.ManualInputInterfaces;
import frc.robot.control.SubsystemCollection;
import frc.robot.subsystems.ControlledFalcon;
import edu.wpi.first.wpilibj2.command.CommandBase;

/** An example command that uses an example subsystem. */
public class ControlledFalconDefualt extends CommandBase {
  @SuppressWarnings({"PMD.UnusedPrivateField", "PMD.SingularField"})
  //private final ExampleSubsystem m_subsystem;

  /**
   * Creates a new ExampleCommand.
   *
   * @param subsystem The subsystem used by this command.
   */

  private SubsystemCollection subsystems;


  private ControlledFalcon motor;
  private double speed = 0.0;
  private boolean done = false;



  public ControlledFalconDefualt(SubsystemCollection collection, ControlledFalcon motor, double speed) {
      this.motor = motor;
      this.speed = speed;

    addRequirements(motor);

    subsystems = collection;


}

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  private void getImputs() {}


  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    motor.setFalconRelitiveSpeed(speed);
    done = true;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done;
  }
}
