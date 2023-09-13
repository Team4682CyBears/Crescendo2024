// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: ManualInputInterfaces.java
// Intent: Forms a class that grants access to driver controlled inputs.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.control;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.*;

//import frc.robot.commands.DriveToPointCommand;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;



public class ManualInputInterfaces {

  // sets joystick variables to joysticks
  private static CommandXboxController driverController = new CommandXboxController(Constants.portDriverController); 
  private static CommandXboxController coDriverController = new CommandXboxController(Constants.portCoDriverController);
  private XboxController coDriverControllerForRumbleOnly = new XboxController(Constants.portCoDriverController);

  // subsystems needed for inputs
  private SubsystemCollection subsystemCollection = null;

  // a member to hold the current game piece target - start with cube because manual likely will target cube as start
 // private ChargedUpGamePiece coDriverControllerGamePieceTarget = ChargedUpGamePiece.Cube;

  /**
   * The constructor to build this 'manual input' conduit
   */
  public ManualInputInterfaces(SubsystemCollection currentCollection){
    subsystemCollection = currentCollection;
    //displayTargetGamePiece();
  }

  /**
   * A method to return the co driver controller for rumble needs
   * @return
   */
  public final XboxController getCoDriverController() {
    return coDriverControllerForRumbleOnly;
  }

  /**
   * A method to get the arcade drive X componet being input from humans
   * @return - a double value associated with the magnitude of the x componet
   */
  public double getInputArcadeDriveX(){
    return driverController.getLeftX();
  }

  /**
   * A method to get the arcade drive X componet being input from humans
   * @return - a double value associated with the magnitude of the x componet
   */
  public double getInputArcadeDriveY(){
    return driverController.getLeftY();
  }

  /**
   * A method to get the spin drive X componet being input from humans
   * @return - a double value associated with the magnitude of the x componet
   */
  public double getInputSpinDriveX(){
    return driverController.getRightX();
  }

  /**
   * A method to get the arcade arm Y componet being input from humans
   * @return - a double value associated with the magnitude of the Y componet
   */
  public double getInputArcadeArmY()
  {
    // use the co drivers right X to represent the horizontal movement
    // and multiply by -1.0 as xbox reports values flipped
    return -1.0 * driverController.getLeftY();
  }

  /**
   * A method to get the arcade arm Z componet being input from humans
   * @return - a double value associated with the magnitude of the Z componet
   */
  public double getInputArcadeArmZ()
  {
    // use the co drivers right Z to represent the vertical movement
    // and multiply by -1.0 as xbox reports values flipped
    return -1.0 * driverController.getRightY();
  }

  /**
   * A method to get the every bot uptake trigger
   * @return - a double value associated with the magnitude of the right trigger pull
   */
  public static double getInputRightTrigger()
  {
    // use the co drivers right trigger
    double inputValue = 0.0;
   
    inputValue = driverController.getRightTriggerAxis();
    return inputValue;
  }

  /**
   * A method to get the every bot expell trigger
   * @return - a double value associated with the magnitude of the left trigger pull
   */
  public static double getInputLeftTrigger()
  {
    // use the co drivers left trigger
    double inputValue = 0.0;
    inputValue = driverController.getLeftTriggerAxis();
    return inputValue;
  }

}
