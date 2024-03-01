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
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.common.ClimberArm;
import frc.robot.common.FeederMode;
import frc.robot.common.TestTrajectories;
import frc.robot.control.InstalledHardware;
import frc.robot.control.ManualInputInterfaces;
import frc.robot.control.SubsystemCollection;
import frc.robot.subsystems.ClimberSubsystem;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.auto.NamedCommands;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.control.Constants;
import frc.robot.subsystems.DrivetrainPowerSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PowerDistributionPanelWatcherSubsystem;
import frc.robot.subsystems.ShooterOutfeedSubsystem;
import frc.robot.subsystems.ShooterAngleSubsystem;

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
    this.initializeShooterOutfeedSubsystem();
    this.initializeShooterAngleSubsystem();

    // init the drivetrain subsystem
    this.initializeDrivetrainSubsystem();

    // init the climber subsystem
    this.initializeClimberSubsystem();

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
    
    // TODO For debugging. Can remove for final competition build. 
    this.initializeDebugDashboard();

    if(this.subsystems.isDriveTrainSubsystemAvailable()) {
      TestTrajectories testtrajectories = new TestTrajectories(this.subsystems.getDriveTrainSubsystem().getTrajectoryConfig());

      SmartDashboard.putData("Basic Forward", new DriveTrajectoryCommand(this.subsystems.getDriveTrainSubsystem(), testtrajectories.traverseSimpleForward));
      SmartDashboard.putData("Forward Arc", new DriveTrajectoryCommand(this.subsystems.getDriveTrainSubsystem(), testtrajectories.traverseForwardArc));
      SmartDashboard.putData("Turn 90", new DriveTrajectoryCommand(this.subsystems.getDriveTrainSubsystem(), testtrajectories.turn90));
    }

    // Path Planner Path Commands
    // commands to drive path planner test trajectories
    // Register Named Commands
    if (this.subsystems.isShooterAngleSubsystemAvailable() && this.subsystems.isShooterAngleSubsystemAvailable() &&
      this.subsystems.isIntakeSubsystemAvailable() && this.subsystems.isFeederSubsystemAvailable()){
      NamedCommands.registerCommand("ShootFromSpeaker",
          new ParallelCommandGroup(
              new ButtonPressCommand("PathPlanner", "ShootFromSpeaker"),
              new ShooterShootCommand(55.0, this.subsystems.getShooterOutfeedSubsystem(),
                  this.subsystems.getShooterAngleSubsystem(), this.subsystems.getFeederSubsystem())));
      NamedCommands.registerCommand("ShootFromNote",
          new ParallelCommandGroup(
              new ButtonPressCommand("PathPlanner", "ShootFromNote"),
              new ShooterShootCommand(40.0, this.subsystems.getShooterOutfeedSubsystem(),
                  this.subsystems.getShooterAngleSubsystem(), this.subsystems.getFeederSubsystem())));
      NamedCommands.registerCommand("IntakeNote",
          new ParallelCommandGroup(
              new ButtonPressCommand("PathPlanner", "IntakeNote"),
              new IntakeAndFeedNoteCommand(this.subsystems.getIntakeSubsystem(), this.subsystems.getFeederSubsystem(),
                  FeederMode.FeedToShooter)));

      PathPlannerPath shootAndMobility = PathPlannerPath.fromPathFile("ShootAndMobility");
      SmartDashboard.putData("ShootAndMobility Path",
          FollowTrajectoryCommandBuilder.build(shootAndMobility, this.subsystems.getDriveTrainSubsystem(), true));

      PathPlannerPath shootPickShoot = PathPlannerPath.fromPathFile("ShootPickShoot");
      SmartDashboard.putData("ShootPickShoot Path",
      new SequentialCommandGroup(
          FollowTrajectoryCommandBuilder.build(shootPickShoot, this.subsystems.getDriveTrainSubsystem(), true),
          new IntakeAndFeedNoteCommand(this.subsystems.getIntakeSubsystem(), this.subsystems.getFeederSubsystem(), FeederMode.FeedToShooter)));

      SmartDashboard.putData("Shoot from speaker",
        new ShooterShootCommand(45.0, this.subsystems.getShooterOutfeedSubsystem(), this.subsystems.getShooterAngleSubsystem(), this.subsystems.getFeederSubsystem()));
    }

    if (this.subsystems.isShooterOutfeedSubsystemAvailable()) {
      SmartDashboard.putData(
          "Spin Up Shooter",
          new ShooterSpinUpCommand(this.subsystems.getShooterOutfeedSubsystem()));
    }

    if (this.subsystems.isShooterAngleSubsystemAvailable()) {
      SmartDashboard.putNumber("Shooter Angle Setter", Constants.shooterStartingAngleOffsetDegrees);
      SmartDashboard.putData(
          "Set Shooter To Specified Angle",
          new ShooterSetAngleTesterCommand(
            () -> SmartDashboard.getNumber("Shooter Angle Setter", Constants.shooterStartingAngleOffsetDegrees),
            this.subsystems.getShooterAngleSubsystem()
          )
      );
    }

    if (this.subsystems.isIntakeSubsystemAvailable() && this.subsystems.isShooterOutfeedSubsystemAvailable()){
      SmartDashboard.putData(
          "Shoot Shooter (at current angle and default speeds)",
          new ShooterShootCommand(Constants.shooterLeftDefaultSpeedRpm, Constants.shooterRightDefaultSpeedRpm, 
          this.subsystems.getShooterOutfeedSubsystem(), this.subsystems.getFeederSubsystem()));
      SmartDashboard.putNumber("Shooter Left Speed RPM Setter", Constants.shooterLeftDefaultSpeedRpm);
      SmartDashboard.putNumber("Shooter Right Speed RPM Setter", Constants.shooterRightDefaultSpeedRpm);
      SmartDashboard.putData(
          "Shooter Shoot to Current Angle and Specified Speeds",
          new ShooterShootCommand(
            () -> SmartDashboard.getNumber("Shooter Left Speed RPM Setter", Constants.shooterLeftDefaultSpeedRpm),
            () -> SmartDashboard.getNumber("Shooter Right Speed RPM Setter", Constants.shooterRightDefaultSpeedRpm),
          this.subsystems.getShooterOutfeedSubsystem(), this.subsystems.getFeederSubsystem()));
    }

    if (this.subsystems.isIntakeSubsystemAvailable()) {
      SmartDashboard.putData(
          "RunIntake",
          new IntakeNoteCommand(this.subsystems.getIntakeSubsystem()));
    }

    if (this.subsystems.isFeederSubsystemAvailable()) {
      SmartDashboard.putData(
          "Run Feeder to Shooter",
          new FeedNoteCommand(this.subsystems.getFeederSubsystem(), FeederMode.FeedToShooter));
    }

    if (this.subsystems.isIntakeSubsystemAvailable() && this.subsystems.isFeederSubsystemAvailable()){
      SmartDashboard.putData(
        "Run Intake and Feeder",
        new IntakeAndFeedNoteCommand(this.subsystems.getIntakeSubsystem(), this.subsystems.getFeederSubsystem(), FeederMode.FeedToShooter));
    }

    if(this.subsystems.isDriveTrainPowerSubsystemAvailable()) {
      SmartDashboard.putData(
        "DriveForwardRobotCentric",
        new DriveTimeCommand(this.subsystems.getDriveTrainSubsystem(),
        new ChassisSpeeds(0.6, 0.0, 0.0),
        3.0));
    }

    if (this.subsystems.isClimberSubsystemAvailable()) {
      SmartDashboard.putData(
        "Climber Left to 10",
        new ClimberArmToHeight(
          this.subsystems.getClimberSubsystem(),
          0.0,
          0.0)
      );
      SmartDashboard.putData(
        "Climber Left to 0",
        new ClimberArmToHeight(
          this.subsystems.getClimberSubsystem(),
          0.0,
          0.0)
      );
    }

  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  /**
   * A method to init the climbers
   */
  private void initializeClimberSubsystem() {
    if((InstalledHardware.leftClimberInstalled && InstalledHardware.leftClimberSensorInstalled) || 
    (InstalledHardware.rightClimberInstalled && InstalledHardware.rightClimberSensorInstalled)) {
      subsystems.setClimberSubsystem(new ClimberSubsystem());
      System.out.println("SUCCESS: ClimberSubsystem");
    }
    else {
      System.out.println("FAIL: ClimberSubsystem");
    }
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
   * A method to init the shooter outfeed
   */
  private void initializeShooterOutfeedSubsystem() {
    if(InstalledHardware.shooterOutfeedInstalled) {
      // The robot's subsystems and commands are defined here...
      subsystems.setShooterOutfeedSubsystem(new ShooterOutfeedSubsystem());
      SmartDashboard.putData("Debug: ShooterSubsystem", subsystems.getShooterOutfeedSubsystem());
      System.out.println("SUCCESS: ShooterOutfeedSubsystem");
    }
    else {
      System.out.println("FAIL: ShooterOutfeedSubsystem");
    }
  }

  /**
   * A method to init the shooter angle
   */
  private void initializeShooterAngleSubsystem() {
    if(InstalledHardware.shooterAngleInstalled) {
      // The robot's subsystems and commands are defined here...
      subsystems.setShooterAngleSubsystem(new ShooterAngleSubsystem());
      SmartDashboard.putData("Debug: ShooterAngleSubsystem", subsystems.getShooterAngleSubsystem());
      System.out.println("SUCCESS: ShooterAngleSubsystem");
    }
    else {
      System.out.println("FAIL: ShooterAngleSubsystem");
    }
  }

  /**
   * A method to late binding of default commands
   */
  private void lateBindDefaultCommands() {
    if(this.subsystems.isManualInputInterfacesAvailable()) {
      // shooter outfeed subsystem default command
      if(this.subsystems.isShooterOutfeedSubsystemAvailable()) {
        this.subsystems.getShooterOutfeedSubsystem().setDefaultCommand(
          new ShooterIdleCommand(
            this.subsystems.getShooterOutfeedSubsystem(),
            this.subsystems.getManualInputInterfaces()));
      }

      // shooter angle subsystem default command
      if(this.subsystems.isShooterAngleSubsystemAvailable()) {
        this.subsystems.getShooterAngleSubsystem().setDefaultCommand(
          new ShooterSetAngleDefaultCommand(
            () -> RobotContainer.getCoDriverRequestedShooterAngle(subsystems),
            this.subsystems.getShooterAngleSubsystem(),
            this.subsystems.getManualInputInterfaces()));
      }

      // climber subsystem default command
      if(this.subsystems.isClimberSubsystemAvailable()) {
        this.subsystems.getClimberSubsystem().setDefaultCommand(
          new ClimberArmDefaultSpeed(
            this.subsystems.getClimberSubsystem(),
            this.subsystems.getManualInputInterfaces(),
            () -> RobotContainer.getCoDriverLeftStickY(this.subsystems),
            () -> RobotContainer.getCoDriverRightStickY(this.subsystems))
        );
      }
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

  /**
   * A wrapper method to obtain the next shooter angle to try
   * @param collection the subsystems in effect here
   * @return a double representing the shooter angle
   */
  private static double getCoDriverRequestedShooterAngle(SubsystemCollection collection) {
      return collection.getManualInputInterfaces().getRequestedShooterAngle();
  }

  /**
   * A method to get the left stick of co driver controller from manual input interfaces
   * @param collection the subsystems in effect here
   * @return - a double representing the stick input
   */
  private static double getCoDriverLeftStickY(SubsystemCollection collection) {
    return collection.getManualInputInterfaces().getCoDriverLeftStickY() * -1.0; // make sure to invert xbox controller Y stick
  }

    /**
   * A method to get the left stick of co driver controller from manual input interfaces
   * @param collection the subsystems in effect here
   * @return - a double representing the stick input
   */
  private static double getCoDriverRightStickY(SubsystemCollection collection) {
    return collection.getManualInputInterfaces().getCoDriverRightStickY() * -1.0; // make sure to invert xbox controller Y stick
  }

}
