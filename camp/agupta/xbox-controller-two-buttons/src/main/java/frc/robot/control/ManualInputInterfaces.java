// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: ManualInputInterfaces.java
// Intent: Forms a class that grants access to driver controlled inputs.
// ************************************************************

// declare package containing class
package frc.robot.control;

// import wpi libraries
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup; // run commands in parallel
import edu.wpi.first.wpilibj2.command.button.CommandXboxController; // button commands for Xbox controller

// import local classes
import frc.robot.commands.RunBagCommand;
import frc.robot.common.BagMode;

// define class
public class ManualInputInterfaces {

  // subsystems needed for inputs
  private SubsystemCollection subsystemCollection = null;

  // sets joystick variables to joysticks
  private CommandXboxController driverController = new CommandXboxController(Constants.portDriverController); 
 
  /**
   * The constructor to build this 'manual input' conduit
   */
  public ManualInputInterfaces(SubsystemCollection currentCollection){
    subsystemCollection = currentCollection;
  }

  /**
   * A method to initialize various commands to the numerous buttons.
   * Need delayed bindings as some subsystems during testing won't always be there.
   */
  public void initializeButtonCommandBindings()
  {
    // Configure the driver xbox controller bindings
    if(InstalledHardware.driverXboxControllerInstalled){ // check if xbox-controller driver installed
      this.bindCommandsToDriverXboxButtons();
    }
  }
  
  /**
   * Will attach commands to the driver Xbox buttons 
   */
  private void bindCommandsToDriverXboxButtons(){

    // check if subsystem is available 
    if(this.subsystemCollection.isBagSubsystemAvailable()) { // runs forward while button is pressed
      // move bag motor forward
      this.driverController.x().onTrue(
        // not used due to both bag commands being under same subsystem
        new ParallelCommandGroup( // allows different subsystems to run in parallel
          new RunBagCommand( // run bag motor
            this.subsystemCollection.getBagSubsystem(), BagMode.Forward)
        )
      );
      // move bag motor in reverse
      this.driverController.b().onTrue( // runs backward while button is pressed
        new ParallelCommandGroup(
          new RunBagCommand( // run bag motor
            this.subsystemCollection.getBagSubsystem(), BagMode.Reverse)
        )
      );
    }
  }
}
