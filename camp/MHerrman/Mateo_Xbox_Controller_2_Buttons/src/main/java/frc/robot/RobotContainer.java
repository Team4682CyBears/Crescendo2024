// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.FeederSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.commands.MotorForward;
import frc.robot.commands.MotorStop;
import frc.robot.commands.MotorBackward;


/**
//The robot's container. Contains our subsystems and commands.
 */
public class RobotContainer {
// The robot's subsystems and commands are defined here...
  private FeederSubsystem feeder =new FeederSubsystem(); //Creates our feeder subsystem. Takes in no arguments.
  private final MotorBackward backwards = new MotorBackward(feeder); //creates our backwards command. Takes in our feeder.
  private final MotorForward forward = new MotorForward(feeder); //creates our forwards command. Takes in our feeder.
  private final MotorStop stop = new MotorStop(feeder); //creates our stop command. Takes in our feeder.

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
  configureBindings();
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
    System.out.println("!!!!!!!!!!!CONFIGURING BUTTON BINDINGS!!!!!!!!!!!!!!");
    this.feeder.setDefaultCommand(stop); // Sets the default command to "stop." This way, when no buttons are being pressed, the motor doesn't move.

    m_driverController.b().whileTrue(backwards); // Binds the "b" button to the backwards command.
    m_driverController.x().whileTrue(forward); // Binds the "x" button to the forward command.
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return new InstantCommand();
  }
}
