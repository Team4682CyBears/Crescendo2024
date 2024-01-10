package frc.robot;

import frc.robot.commands.Autos;
import frc.robot.commands.NeoShootAllStopCommand;
import frc.robot.commands.NeoShootAtSpeedCommand;
import frc.robot.commands.TalonShootAllStopCommand;
import frc.robot.commands.TalonShootAtSpeedCommand;
import frc.robot.subsystems.TalonShooterSubsystem;
import frc.robot.subsystems.NeoShooterSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class RobotContainer {

  // *****************************************************
  // ADJUST THIS IF YOU WANT TALON or NEO SETUP
  // 1. For single TALON, set - currentTargetIsTalon = true;
  // 2. For quad NEO, set - currentTargetIsTalon = false;
  // *****************************************************
  private final boolean currentTargetIsTalon = true;

  // The robot's subsystems and commands are defined here...
  private TalonShooterSubsystem talonShooterSubsystem;
  private TalonShootAtSpeedCommand talonSpeedCommand;
  private NeoShooterSubsystem neoShooterSubsystem;
  private NeoShootAtSpeedCommand neoSpeedCommand;

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController driverController = new CommandXboxController(Constants.kDriverControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {

    // init shooter and default commands
    if(currentTargetIsTalon) {
      talonShooterSubsystem = new TalonShooterSubsystem();
      talonShooterSubsystem.setDefaultCommand(new TalonShootAllStopCommand(talonShooterSubsystem));
      talonSpeedCommand = new TalonShootAtSpeedCommand(talonShooterSubsystem);
    }
    else {
      neoShooterSubsystem = new NeoShooterSubsystem();
      neoShooterSubsystem.setDefaultCommand(new NeoShootAllStopCommand(neoShooterSubsystem));
      neoSpeedCommand = new NeoShootAtSpeedCommand(neoShooterSubsystem);
    }
  
    // Configure the trigger bindings
    configureBindings();
  }
  
  private void configureBindings() {
    if(currentTargetIsTalon) {
      // a button drives the single motor talon setup
      driverController.a().whileTrue(this.talonSpeedCommand);
    }
    else {
      // a button drives the four motor neo setup
      driverController.a().whileTrue(this.neoSpeedCommand);
    }
  }

  public Command getAutonomousCommand() {
    // auto for talon 
    if(currentTargetIsTalon) {
      return Autos.talonAuto(talonShooterSubsystem);
    }
    // auto for neo
    else {
      return Autos.neoAuto(neoShooterSubsystem);
    }
  }

}
