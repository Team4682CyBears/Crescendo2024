package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.TalonShootAtSpeedCommand;
import frc.robot.subsystems.TalonShooterSubsystem;
import frc.robot.subsystems.NeoShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {

  // The robot's subsystems and commands are defined here...
  private final TalonShooterSubsystem talonShooterSubsystem = new TalonShooterSubsystem();
  private final NeoShooterSubsystem neoShooterSubsystem = new NeoShooterSubsystem();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController =
      new CommandXboxController(Constants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();
  }

  private void configureBindings() {
    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    driverController.b().whileTrue(new TalonShootAtSpeedCommand(talonShooterSubsystem));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(talonShooterSubsystem);
  }
}
