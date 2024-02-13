// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: IntakeSubsystem.java
// Intent: Forms the prelminary code for intake subsystem.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.control.Constants;

/**
 * Forms a class for the intake subsystem consisting of
 * a Neo motor and a TOF sensor
 */
public class IntakeSubsystem extends SubsystemBase {
  //Neo motor
  private CANSparkMax intakeMotor = new CANSparkMax(Constants.intakeMotorCanId, MotorType.kBrushless);
  //BeamBreak sensor
  private TofSubsystem beambreakSensor = new TofSubsystem(Constants.intakeTofCanId);

  /**
   * Constructor for the IntakeSubsystem
   */
  public IntakeSubsystem() {
  }

  /**
   * A method to detect the presence of a note
   * @return true if the note is detected
   */
  public boolean isNoteDetected(){
    return beambreakSensor.isNoteDetected();
  }

  /**
   * periodic for this subsystem. Called once per scheduler run (20ms) 
   */
  @Override
  public void periodic() {
  }

  /**
   * A method to set the intake speed
   * @param speed (between -1 and 1)
   */
  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  /**
   * A method to stop the intake subsystem
   */
  public void setAllStop() {
    intakeMotor.set(0.0);
  }

  /**
   * This method will be called once per scheduler run during simulation
   */
  @Override
  public void simulationPeriodic() {
  }

}