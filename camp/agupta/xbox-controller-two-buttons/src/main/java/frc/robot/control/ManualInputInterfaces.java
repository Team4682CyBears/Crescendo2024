// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: ManualInputInterfaces.java
// Intent: Forms a class that grants access to driver controlled inputs.
// ************************************************************
package frc.robot.control;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.ButtonPressCommand;
import frc.robot.commands.FeederLaunchNote;
import frc.robot.commands.RewindFeederCommand;
import frc.robot.common.FeederMode;

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
      this.bindCommandsToDriverXBboxButtons();
    }

  }
  
  /**
   * Will attach commands to the Driver XBox buttons 
   */
  private void bindCommandsToDriverXBboxButtons(){
    if(InstalledHardware.driverXboxControllerInstalled){ // check if xbox-controller driver installed

      if(this.subsystemCollection.isFeederSubsystemAvailable()) {
        // angle change commands 
        // forward
        this.driverController.x().whileTrue(
          new ParallelCommandGroup(
            new FeederLaunchNote(
              subsystemCollection.getFeederSubsystem(), FeederMode.FeedToShooter, Constants.feederLaunchTimeoutSecondsInTele)
          )
        );
        // backward
        this.driverController.b().whileTrue(
          new ParallelCommandGroup(
            new RewindFeederCommand(
                this.subsystemCollection.getFeederSubsystem(), 
                FeederMode.FeedToShooter
            )
            //new ButtonPressCommand(
              //"driverController.y()",
              //"Remove Note")
              //)
          )
        );
   
      }
    }
  }
}
