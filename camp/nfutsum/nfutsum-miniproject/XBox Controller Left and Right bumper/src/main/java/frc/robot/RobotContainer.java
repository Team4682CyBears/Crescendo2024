// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Stop;
import frc.robot.commands.Back;
import frc.robot.commands.Forward;
import frc.robot.subsystems.Motor;
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
  private final Motor m_motor = new Motor();

  private final Forward theForward = new Forward(m_motor);
  private final Stop theStop = new Stop(m_motor);
  private final Back theBackward = new Back(m_motor);
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

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
    /*/ Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_motor::exampleCondition)
        .onTrue(new forward(m_motor));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.*/
    m_driverController.leftBumper().onTrue(theBackward);
    m_driverController.leftBumper().onFalse(theStop);
    m_driverController.rightBumper().onTrue(theForward);
    m_driverController.rightBumper().onFalse(theStop);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  /*public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return stop.exampleAuto(m_exampleSubsystem);
  }*/
}
