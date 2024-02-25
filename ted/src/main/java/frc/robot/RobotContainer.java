// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ShootAllStopCommand;
import frc.robot.commands.ShooterSetAngleCommand;
import frc.robot.commands.ShooterSetAngleTesterCommand;
import frc.robot.commands.ShooterShootCommand;
import frc.robot.commands.ShooterSpinUpCommand;
import frc.robot.common.FeederMode;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.control.Constants;
import frc.robot.control.InstalledHardware;
import frc.robot.control.ManualInputInterfaces;
import frc.robot.control.SubsystemCollection;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.subsystems.DrivetrainPowerSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PowerDistributionPanelWatcherSubsystem;
import frc.robot.subsystems.TalonShooterSubsystem;
import frc.robot.commands.DriveTimeCommand;
import frc.robot.commands.FeedNoteCommand;
import frc.robot.commands.IntakeNoteCommand;

public class RobotContainer {

  private SubsystemCollection subsystems = new SubsystemCollection();

  public RobotContainer() {

    // init the pdp watcher
    this.initializePowerDistributionPanelWatcherSubsystem();

        // intake subsystem init
    this.initializeIntakeSubsystem();

    // feeder subsystem init
    this.initializeFeederSubsystem();

    // shooter subsystem init
    this.initializeShooterSubsystem();

    // init the various subsystems
    this.initializeDrivetrainSubsystem();

    // init the input system 
    this.initializeManualInputInterfaces();

    // do late binding of default commands
    this.lateBindDefaultCommands();

    // Configure the button bindings
    if(this.subsystems.isManualInputInterfacesAvailable()) {
      System.out.println(">>>> Initializing button bindings.");
      this.subsystems.getManualInputInterfaces().initializeButtonCommandBindings();
      System.out.println(">>>> Finished initializing button bindings.");
    }

    // Put command scheduler on dashboard
    SmartDashboard.putData(CommandScheduler.getInstance());

    if(this.subsystems.isDriveTrainSubsystemAvailable()) {
      SmartDashboard.putData(
        "DriveForwardRobotCentric",
        new DriveTimeCommand(this.subsystems.getDriveTrainSubsystem(),
        new ChassisSpeeds(0.6, 0.0, 0.0),
        3.0));
    }

    if (this.subsystems.isShooterSubsystemAvailable()) {
      SmartDashboard.putData(
          "Spin Up Shooter",
          new ShooterSpinUpCommand(this.subsystems.getShooterSubsystem()));
      SmartDashboard.putNumber("Shooter Angle Setter", Constants.shooterStartingAngleOffsetDegrees);
      SmartDashboard.putData(
          "Set Shooter To Specified Angle",
          new ShooterSetAngleTesterCommand(
            () -> SmartDashboard.getNumber("Shooter Angle Setter", Constants.shooterStartingAngleOffsetDegrees),
            this.subsystems.getShooterSubsystem()
          )
      );
      // put shooter subsystem status on dashboard
      SmartDashboard.putData(this.subsystems.getShooterSubsystem());

    }

    if (this.subsystems.isIntakeSubsystemAvailable() && this.subsystems.isShooterSubsystemAvailable()){
      SmartDashboard.putData(
          "Shoot Shooter (at current angle and default speeds)",
          new ShooterShootCommand(Constants.shooterLeftDefaultSpeedRpm, Constants.shooterRightDefaultSpeedRpm, this.subsystems.getShooterSubsystem(), this.subsystems.getFeederSubsystem()));
      SmartDashboard.putNumber("Shooter Left Speed RPM Setter", Constants.shooterLeftDefaultSpeedRpm);
      SmartDashboard.putNumber("Shooter Right Speed RPM Setter", Constants.shooterRightDefaultSpeedRpm);
      SmartDashboard.putData(
          "Shooter Shoot to Current Angle and Specified Speeds",
          new ShooterShootCommand(
            () -> SmartDashboard.getNumber("Shooter Left Speed RPM Setter", Constants.shooterLeftDefaultSpeedRpm),
            () -> SmartDashboard.getNumber("Shooter Right Speed RPM Setter", Constants.shooterRightDefaultSpeedRpm),
          this.subsystems.getShooterSubsystem(), this.subsystems.getFeederSubsystem()));
    }

    if (this.subsystems.isIntakeSubsystemAvailable()) {
      SmartDashboard.putData(
          "Run Intake",
          new IntakeNoteCommand(this.subsystems.getIntakeSubsystem()));
    }

    if (this.subsystems.isFeederSubsystemAvailable()) {
      SmartDashboard.putData(
          "Run Feeder to Shooter",
          new FeedNoteCommand(this.subsystems.getFeederSubsystem(), FeederMode.FeedToShooter));
    }

    if(this.subsystems.isDriveTrainPowerSubsystemAvailable()) {
      SmartDashboard.putData(
        "DriveForwardRobotCentric",
        new DriveTimeCommand(this.subsystems.getDriveTrainSubsystem(),
        new ChassisSpeeds(0.6, 0.0, 0.0),
        3.0));
    }
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  /**
   * A method to init the PDP watcher
   */
  private void initializePowerDistributionPanelWatcherSubsystem() {
    if(InstalledHardware.powerDistributionPanelInstalled) {
      subsystems.setPowerDistributionPanelWatcherSubsystem(new PowerDistributionPanelWatcherSubsystem());
      System.out.println("SUCCESS: initializePowerDistributionPanelWatcherSubsystem");
    }
    else {
      System.out.println("FAIL: initializePowerDistributionPanelWatcherSubsystem");
    }
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


    }
    else {
      System.out.println("FAIL: ShooterSubsystem");
    }
  }

  /**
   * A method to late binding of default commands
   */
  private void lateBindDefaultCommands() {

    // shooter subsystem default commands
    if(this.subsystems.isShooterSubsystemAvailable() && 
    this.subsystems.isManualInputInterfacesAvailable()) {
      System.out.println("********************* GOT TO SHOOTER DEFAULT COMMAND *******************");
      // Set up the default command for the shooter.
      this.subsystems.getShooterSubsystem().setDefaultCommand(
        new SequentialCommandGroup(
          new ShootAllStopCommand(this.subsystems.getShooterSubsystem()),
          new ShooterSetAngleTesterCommand(
            () -> this.subsystems.getManualInputInterfaces().getInputShooterAngle(),
            this.subsystems.getShooterSubsystem())));
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
  //TODO create climber arms in InstalledHardware
  //TODO create climber arms subsystem
  
  /*private void initializeClimberArmsSubsystem() {
    if(InstalledHardware.climberArmMotorInstalled) {
      // The robot's subsystems and commands are defined here...
      subsystems.setClimberArmsSubsystem(new ClimberArmsSubsystem());
      SmartDashboard.putData("Debug: ClimerArmSub", subsystems.getClimberArmsSubsystem());
      System.out.println("SUCCESS: initializeClimberArm");

      // Set up the default command for the arm.
      // Left stick Y axis -> vertical arm in / out movement
      subsystems.getClimberArmsSubsystem().setDefaultCommand(new DefaultArmCommand(
        subsystems.getClimberArmsSubsystem(),
        () -> subsystems.getManualInputInterfaces().getInputClimberArmsZ()
      ));
    }
    else {
      System.out.println("FAIL: initializeArms");
    }
  }*/

}
