// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// File: ControlledFalcon.java
// Intent: Holds commands to interface with a falcon motor.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;
import frc.robot.Constants;
import frc.robot.commands.ControlledFalconDefault;
import frc.robot.control.ManualInputInterfaces;
import frc.robot.control.SubsystemCollection;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

public class ControlledFalcon extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */

    TalonFX motor = new TalonFX(Constants.portControlledFalcon);
    TalonFXConfiguration config = new TalonFXConfiguration();

   private SubsystemCollection subsystemCollection;
   private final ControlledFalcon falc = new ControlledFalcon();

  public ControlledFalcon() {
    setDefaultCommand(new ControlledFalconDefault(subsystemCollection, falc, ManualInputInterfaces.getInputLeftTrigger(), ManualInputInterfaces.getInputRightTrigger()));

    config.supplyCurrLimit.enable = true;
    config.supplyCurrLimit.triggerThresholdCurrent = 40; // the peak supply current, in amps
    config.supplyCurrLimit.triggerThresholdTime = 1.5; // the time at the peak supply current before the limit triggers, in sec
    config.supplyCurrLimit.currentLimit = 30; // the current to maintain if the peak supply limit is triggered
    motor.configAllSettings(config); // apply the config settings; this selects the quadrature encoder
  }

  public void setFalconRelativeSpeed(double controlledFalconSpeed) {
    motor.set(TalonFXControlMode.PercentOutput, controlledFalconSpeed);
    System.out.print(controlledFalconSpeed);
  }  
  


  @Override
  public void periodic() {
  }


}
