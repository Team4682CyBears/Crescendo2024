// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// File: ManualInputInterfaces.java
// Intent: Forms a class that grants access to driver controlled inputs.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.control;

import frc.robot.*;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;


public class ManualInputInterfaces {

  // sets joystick variables to joysticks
  private static CommandXboxController driverController = new CommandXboxController(Constants.portDriverController); 

  // subsystems needed for inputs
  private SubsystemCollection subsystemCollection = null;


  public ManualInputInterfaces(SubsystemCollection currentCollection){
    subsystemCollection = currentCollection;
  }

  public static double getInputRightTrigger()
  {
    // use the co drivers right trigger
    double inputValue = 0.0;
   
    inputValue = driverController.getRightTriggerAxis();
    return inputValue;
  }

  public static double getInputLeftTrigger()
  {
    // use the co drivers left trigger
    double inputValue = 0.0;
    inputValue = driverController.getLeftTriggerAxis();
    return inputValue;
  }


}
