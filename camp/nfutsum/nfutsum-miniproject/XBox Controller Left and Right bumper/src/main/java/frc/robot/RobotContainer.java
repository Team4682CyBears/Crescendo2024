// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: RobotContainer.java
// Intent: Connects commands to controller
// ************************************************************

package frc.robot;

import frc.robot.commands.MotorStopCommand;
import frc.robot.commands.MotorBackwardCommand;
import frc.robot.commands.MotorForwardCommand;
import frc.robot.subsystems.MotorSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;


/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final MotorSubsystem motor = new MotorSubsystem();

  private final MotorForwardCommand theForward = new MotorForwardCommand(motor);
  private final MotorStopCommand theStop = new MotorStopCommand(motor);
  private final MotorBackwardCommand theBackward = new MotorBackwardCommand(motor);
  
  // Declares Xbox Controller
  private final CommandXboxController driverController =
      new CommandXboxController(Constants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  public String robotContainer(String inputDeviceDescription, String inputActionDescription) {
    // Configure the trigger bindings
      String inputDevice = inputDeviceDescription;
      String inputAction = inputActionDescription;
      return inputAction + inputDevice;
  }
/**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // Schedule Forward or Back command when the Xbox controller's left bumper or right bumper are pressed,
    // cancelling on release.*/
    driverController.leftBumper().onTrue(theBackward);
    driverController.leftBumper().onFalse(theStop);
    driverController.rightBumper().onTrue(theForward);
    driverController.rightBumper().onFalse(theStop);
  }

}
