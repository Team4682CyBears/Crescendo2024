// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.common.TestTrajectories;
import frc.robot.common.SwerveTrajectoryConfig;
import frc.robot.common.FeederMode;
import frc.robot.control.InstalledHardware;
import frc.robot.control.ManualInputInterfaces;
import frc.robot.control.SubsystemCollection;
import frc.robot.subsystems.*;
import frc.robot.commands.*;

public class RobotContainer {

  private SubsystemCollection subsystems = new SubsystemCollection();

  public RobotContainer() {

    // init the pdp watcher
    this.initializePowerDistributionPanelWatcherSubsystem();

    // init the camera (before drivetrain)
    this.initializeCameraSubsystem();

    // init the various subsystems
    this.initializeDrivetrainSubsystem();

    // init the input system 
    this.initializeManualInputInterfaces();

    // intake subsystem init
    this.initializeIntakeSubsystem();

    // feeder subsystem init
    this.initializeFeederSubsystem();

    // shooter subsystem init
    this.initializeShooterSubsystem();

    // steer motor subsystem init
    this.initializeSteerMotorSubsystem();

    // steer motor can coder subsystem init
    this.initializeSteerMotorCanCoderSubsystem();

    // Configure the button bindings
    System.out.println(">>>> Initializing button bindings.");
    this.subsystems.getManualInputInterfaces().initializeButtonCommandBindings();
    System.out.println(">>>> Finished initializing button bindings.");

    SmartDashboard.putData(
      "DriveForwardRobotCentric",
      new DriveTimeCommand(this.subsystems.getDriveTrainSubsystem(),
      new ChassisSpeeds(0.6, 0.0, 0.0),
      3.0));

    if (InstalledHardware.shooterInstalled) {
      SmartDashboard.putData(
          "Spin Up Shooter",
          new ShooterSpinUpCommand(this.subsystems.getShooterSubsystem()));
    }

    if (InstalledHardware.intakeInstalled) {
      SmartDashboard.putData(
          "Run Intake",
          new IntakeNoteCommand(this.subsystems.getIntakeSubsystem()));
    }

    if (InstalledHardware.feederInstalled) {
      SmartDashboard.putData(
          "Run Feeder to Shooter",
          new FeedNoteCommand(this.subsystems.getFeederSubsystem(), FeederMode.FeedToShooter));
    }

    if(this.subsystems.getDriveTrainPowerSubsystem() != null) {
      SmartDashboard.putData(
        "DriveForwardRobotCentric",
        new DriveTimeCommand(this.subsystems.getDriveTrainSubsystem(),
        new ChassisSpeeds(0.6, 0.0, 0.0),
        3.0));
    }

    final TestTrajectories testTrajectories = new TestTrajectories(subsystems.getDriveTrainSubsystem().getTrajectoryConfig());
    
    SmartDashboard.putData("Allign relative to tag", 
      new AllignRelativeToTagCommand(this.subsystems.getDriveTrainSubsystem(), this.subsystems.getCameraSubsystem(), new Pose2d(new Translation2d(1.0, 0.0), new Rotation2d(0.0)), 7.0));

    SmartDashboard.putData("Drive Forward Trajectory",
      new DriveTrajectoryCommand(subsystems.getDriveTrainSubsystem(), testTrajectories.traverseSimpleForward));

    SmartDashboard.putData("Drive ZigZag Trajectory",
      new DriveTrajectoryCommand(subsystems.getDriveTrainSubsystem(), testTrajectories.traverseZigZag));

    SmartDashboard.putData("Drive Turn90 Trajectory",
      new DriveTrajectoryCommand(subsystems.getDriveTrainSubsystem(), testTrajectories.turn90));
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
      subsystems.setDriveTrainSubsystem(new DrivetrainSubsystem(subsystems));
      subsystems.getDriveTrainSubsystem().zeroRobotPosition(); // can I add this?
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
   * A method to init the Limelight
   */
  private void initializeCameraSubsystem(){
    if(InstalledHardware.limelightInstalled) {
      subsystems.setCameraSubsystem(new CameraSubsystem());
      System.out.println("SUCCESS: initializeCamera");
    }
    else {
      System.out.println("FAIL: initializeCamera");
    }
  }
  
  /**
   * A method to init the feeder subsystem
   */
  private void initializeFeederSubsystem(){
    if(InstalledHardware.feederInstalled){
      subsystems.setFeederSubsystem(new FeederSubsystem());

      // default command for feeder is to stop
      subsystems.getFeederSubsystem().setDefaultCommand(
        new InstantCommand(
          subsystems.getFeederSubsystem()::setAllStop, 
          subsystems.getFeederSubsystem()));
      System.out.println("SUCCESS: FeederSubsystem");
    } else {
      System.out.println("FAIL: FeederSubsystem");
    }
  }

  /**
   * A method to init the intake subsystem
   */
  private void initializeIntakeSubsystem(){
    if(InstalledHardware.intakeInstalled){
      subsystems.setIntakeSubsystem(new IntakeSubsystem());

      // default command for intake is to stop
      subsystems.getIntakeSubsystem().setDefaultCommand(
        new InstantCommand(
          subsystems.getIntakeSubsystem()::setAllStop, 
          subsystems.getIntakeSubsystem()));
      System.out.println("SUCCESS: IntakeSubsystem");
    } else {
      System.out.println("FAIL: IntakeSubsystem");
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
   * A method to init the shooter
   */
  private void initializeShooterSubsystem() {
    if(InstalledHardware.shooterInstalled) {
      // The robot's subsystems and commands are defined here...
      subsystems.setShooterSubsystem(new TalonShooterSubsystem());
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

  /**
   * A method to init the steer motor subsystem
   */
  private void initializeSteerMotorSubsystem() {
    if(InstalledHardware.leftFrontDriveInstalledForTesting) {
      // The robot's subsystems and commands are defined here...
      subsystems.setSteerMotorSubsystem(new SteerMotorSubsystem());
      SmartDashboard.putData("Debug: SteerMotorSubsystem", subsystems.getSteerMotorSubsystem());
      System.out.println("SUCCESS: SteerMotorSubsystem");
    }
    else {
      System.out.println("FAIL: SteerMotorSubsystem");
    }
  }

  /**
   * A method to init the steer motor subsystem
   */
  private void initializeSteerMotorCanCoderSubsystem() {
    if(InstalledHardware.leftFrontDriveCanCoderInstalledForTesting) {
      // The robot's subsystems and commands are defined here...
      subsystems.setSteerMotorCanCoderSubsystem(new SteerMotorCanCoderSubsystem());
      SmartDashboard.putData("Debug: SteerMotorCanCoderSubsystem", subsystems.getSteerMotorCanCoderSubsystem());
      System.out.println("SUCCESS: SteerMotorCanCoderSubsystem");
    }
    else {
      System.out.println("FAIL: SteerMotorCanCoderSubsystem");
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
