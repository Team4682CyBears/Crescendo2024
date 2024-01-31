// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.ShootAllStopCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.control.InstalledHardware;
import frc.robot.control.ManualInputInterfaces;
import frc.robot.control.SubsystemCollection;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DrivetrainPowerSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.PowerDistributionPanelWatcherSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class RobotContainer {

  private SubsystemCollection subsystems = new SubsystemCollection();

  public RobotContainer() {
    // init the pdp watcher
    this.initializePowerDistributionPanelWatcherSubsystem();

    // init the various subsystems
    this.initializeDrivetrainSubsystem();

    // init the input system 
    this.initializeManualInputInterfaces();

    // shooter subsystem init - later
    this.initializeShooterSubsystem();
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  /**
   * A method to init the PDP watcher
   */
  private void initializePowerDistributionPanelWatcherSubsystem() {
    subsystems.setPowerDistributionPanelWatcherSubsystem(new PowerDistributionPanelWatcherSubsystem());
    System.out.println("SUCCESS: initializePowerDistributionPanelWatcherSubsystem");
  }

  /**
   * A method to init items for the debug dashboard
   */
  private void initializeDebugDashboard() {
    SmartDashboard.putData("Debug: CommandScheduler", CommandScheduler.getInstance());
  }

  /**
   * A method to init the drive train
   */
  private void initializeDrivetrainSubsystem() {
    if(InstalledHardware.leftFrontDriveInstalled && 
      InstalledHardware.leftRearDriveInstalled && 
      InstalledHardware.rightFrontDriveInstalled &&
      InstalledHardware.rightRearDriveInstalled &&
      InstalledHardware.navxInstalled) {
      // The robot's subsystems and commands are defined here...
      subsystems.setDriveTrainSubsystem(new DrivetrainSubsystem());
      subsystems.setDriveTrainPowerSubsystem(new DrivetrainPowerSubsystem(subsystems.getDriveTrainSubsystem()));
      SmartDashboard.putData("Debug: DrivetrainSub", subsystems.getDriveTrainSubsystem());
      System.out.println("SUCCESS: initializeDrivetrain");

      // Set up the default command for the drivetrain.
      // The controls are for field-oriented driving:
      // Left stick Y axis -> forward and backwards movement
      // Left stick X axis -> left and right movement
      // Right stick X axis -> rotation
      subsystems.getDriveTrainSubsystem().setDefaultCommand(
        new DefaultDriveCommand(
          subsystems.getDriveTrainSubsystem(),
          () -> -RobotContainer.modifyAxisSquare(subsystems.getManualInputInterfaces().getInputArcadeDriveY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
          () -> -RobotContainer.modifyAxisSquare(subsystems.getManualInputInterfaces().getInputArcadeDriveX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
          () -> -RobotContainer.modifyAxisSquare(subsystems.getManualInputInterfaces().getInputSpinDriveX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
          ));

    }
    else {
      System.out.println("FAIL: initializeDrivetrain");
    }
  }
  
  /**
   * A method to init the input interfaces
   */
  private void initializeManualInputInterfaces() {
    // note: in this case it is safe to build the interfaces if only one of the controllers is present
    // because button binding assignment code checks that each is installed later (see: initializeButtonCommandBindings)
    if(InstalledHardware.driverXboxControllerInstalled ||
      InstalledHardware.coDriverXboxControllerInstalled) {
      subsystems.setManualInputInterfaces(new ManualInputInterfaces(subsystems));
      System.out.println("SUCCESS: initializeManualInputInterfaces");
    }
    else {
      System.out.println("FAIL: initializeManualInputInterfaces");
    }
  }

  /**
   * A method to init the arm
   */
  private void initializeShooterSubsystem() {
    if(InstalledHardware.shooterInstalled) {
      // The robot's subsystems and commands are defined here...
      subsystems.setShooterSubsystem(new ShooterSubsystem());
      SmartDashboard.putData("Debug: ShooterSubsystem", subsystems.getShooterSubsystem());
      System.out.println("SUCCESS: ShooterSubsystem");

      // Set up the default command for the arm.
      // Left stick X axis -> horizontal arm in / out movement
      // Left stick Y axis -> vertical arm in / out movement
      subsystems.getShooterSubsystem().setDefaultCommand(new ShootAllStopCommand(
        subsystems.getShooterSubsystem()));
    }
    else {
      System.out.println("FAIL: ShooterSubsystem");
    }
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      }
      else {
        return (value + deadband) / (1.0 - deadband);
      }
    }
    else {
      return 0.0;
    }
  }

  private static double modifyAxisSquare(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Joystick input exponent
    value = Math.copySign(value * value, value);

    return value;
  }
}
