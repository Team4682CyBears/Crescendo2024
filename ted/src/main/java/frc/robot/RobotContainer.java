// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: RobotContainer.java
// Intent: main robot body
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot;

import com.ctre.phoenix.led.CANdle.LEDStripType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.common.TestTrajectories;
import frc.robot.common.FeederMode;
import frc.robot.common.LEDState;
import frc.robot.control.InstalledHardware;
import frc.robot.control.ManualInputInterfaces;
import frc.robot.control.SubsystemCollection;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import frc.robot.control.AutonomousChooser;
import frc.robot.control.Constants;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;

public class RobotContainer {

  private SubsystemCollection subsystems = new SubsystemCollection();
  private final AutonomousChooser autonomousChooser;

  public RobotContainer() {

    // init the data logging
    this.initializeDataLogging();

    // init the pdp watcher
    this.initializePowerDistributionPanelWatcherSubsystem();

    // init the camera (before drivetrain)
    this.initializeCameraSubsystem();

    // init the leds
    this.initializeLEDSubsystem();

    // intake subsystem init
        // intake subsystem init
    this.initializeIntakeSubsystem();

    // feeder subsystem init
    this.initializeFeederSubsystem();

    // shooter subsystem init
    this.initializeShooterOutfeedSubsystem();
    this.initializeShooterAngleSubsystem();

    // init the various subsystems
    this.initializeDrivetrainSubsystem();

    // init the climber subsystem
    this.initializeClimberSubsystem();

    // init the input system 
    this.initializeManualInputInterfaces();

    // do late binding of default commands
    this.lateBindDefaultCommands();

    // bind brownout actions
    this.bindBrownoutActions();

    AutonomousChooser.configureAutoBuilder(subsystems);
    autonomousChooser  = new AutonomousChooser(subsystems);


    // Configure the button bindings
    if(this.subsystems.isManualInputInterfacesAvailable()) {
      DataLogManager.log(">>>> Initializing button bindings.");
      this.subsystems.getManualInputInterfaces().initializeButtonCommandBindings();
      DataLogManager.log(">>>> Finished initializing button bindings.");
    }
    
    // TODO For debugging. Can remove for final competition build. 
    this.initializeDebugDashboard();

    if (subsystems.isDriveTrainSubsystemAvailable()) {
      TestTrajectories testtrajectories = new TestTrajectories(
          this.subsystems.getDriveTrainSubsystem().getTrajectoryConfig());

      SmartDashboard.putData("Basic Forward",
          new DriveTrajectoryCommand(this.subsystems.getDriveTrainSubsystem(), testtrajectories.traverseSimpleForward));
      SmartDashboard.putData("Forward Arc",
          new DriveTrajectoryCommand(this.subsystems.getDriveTrainSubsystem(), testtrajectories.traverseForwardArc));
      SmartDashboard.putData("Turn 90",
          new DriveTrajectoryCommand(this.subsystems.getDriveTrainSubsystem(), testtrajectories.turn90));
    }

    // Path Planner Path Commands
    // commands to drive path planner test trajectories
    // Register Named Commands

      //Command shootPickShootAuto = AutoBuilder.buildAuto("ShootPickShoot");
      //SmartDashboard.putData("ShootPickShoot Auto", shootPickShootAuto);

      //Command sourceSideWingAuto = AutoBuilder.buildAuto("SourceSideWing");
      //SmartDashboard.putData("SourceSideWing Auto", sourceSideWingAuto);

      //Command oneTwoThreeSourceSideAuto = AutoBuilder.buildAuto("123SourceSide");
      //SmartDashboard.putData("123SourceSide Auto", oneTwoThreeSourceSideAuto);

    // Put command scheduler on dashboard
    SmartDashboard.putData(CommandScheduler.getInstance());

    // this should be disabled during competition as it sometimes crashes shuffleboard
    // disable by setting setShooterAngleFromShuffleboard in InstalledHardware
    if (this.subsystems.isShooterAngleSubsystemAvailable() && InstalledHardware.setShooterAngleFromShuffleboard) {
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
          new ShooterShootCommand(Constants.shooterDefaultSpeedRpm, 
          this.subsystems.getShooterOutfeedSubsystem(), this.subsystems.getFeederSubsystem()));
      SmartDashboard.putNumber("Shooter Speed RPM Setter", Constants.shooterDefaultSpeedRpm);
      SmartDashboard.putData(
          "Shooter Shoot to Current Angle and Specified Speeds",
          new ShooterShootCommand(
            () -> SmartDashboard.getNumber("Shooter Speed RPM Setter", Constants.shooterDefaultSpeedRpm),
          this.subsystems.getShooterOutfeedSubsystem(), this.subsystems.getFeederSubsystem()));
      SmartDashboard.putData(
          "Spin Up Shooter at specified speeds",
          new ShooterSpinUpCommand(this.subsystems.getShooterOutfeedSubsystem(), () -> SmartDashboard.getNumber("Shooter Speed RPM Setter", Constants.shooterDefaultSpeedRpm)));
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
    return autonomousChooser.getCommand();
  }

  private void bindBrownoutActions(){
    if (Constants.enableBrownoutActions){
      this.subsystems.getPowerDistributionPanelWatcherSubsystem().setBrownoutCallback(
        new InstantCommand(() -> this.subsystems.getDriveTrainAccelerationSubsystem().setReducedReductionFactor()),
        Constants.brownoutEventsBeforeAction);
    }
  }

  /**
   * A method to init the climbers
   */
  private void initializeClimberSubsystem() {
    if((InstalledHardware.leftClimberInstalled && InstalledHardware.leftClimberSensorInstalled) || 
    (InstalledHardware.rightClimberInstalled && InstalledHardware.rightClimberSensorInstalled)) {
      subsystems.setClimberSubsystem(new ClimberSubsystem());
      DataLogManager.log("SUCCESS: ClimberSubsystem");
      subsystems.getClimberSubsystem().setDefaultCommand(
        new ClimberArmDefaultSpeed(
          subsystems.getClimberSubsystem(),
          () -> RobotContainer.deadband(this.subsystems.getManualInputInterfaces().getInputLeftClimberArmZ(), Constants.climberControllerStickDeadband),
          () -> RobotContainer.deadband(this.subsystems.getManualInputInterfaces().getInputRightClimberArmZ(), Constants.climberControllerStickDeadband)));
      SmartDashboard.putData("Left Climber",   new ClimberArmDefaultSpeed(subsystems.getClimberSubsystem(), () -> -.2, () -> 0));
      SmartDashboard.putData("Right Climber",   new ClimberArmDefaultSpeed(subsystems.getClimberSubsystem(), () -> 0, () -> -.2));
    }
    else {
      DataLogManager.log("FAIL: ClimberSubsystem");
    }
  }

   /**
   * A method to init all the data logging
   */
  private void initializeDataLogging() {
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog());
  }

  /**
   * A method to init the PDP watcher
   */
  private void initializePowerDistributionPanelWatcherSubsystem() {
    if(InstalledHardware.powerDistributionPanelInstalled) {
      subsystems.setPowerDistributionPanelWatcherSubsystem(new PowerDistributionPanelWatcherSubsystem());
      DataLogManager.log("SUCCESS: initializePowerDistributionPanelWatcherSubsystem");
    }
    else {
      DataLogManager.log("FAIL: initializePowerDistributionPanelWatcherSubsystem");
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
      subsystems.setDriveTrainSubsystem(new DrivetrainSubsystem(subsystems));
      subsystems.getDriveTrainSubsystem().zeroRobotPosition(); // can I add this?
      subsystems.setDriveTrainPowerSubsystem(new DrivetrainPowerSubsystem(subsystems.getDriveTrainSubsystem()));
      subsystems.setDriveTrainAccelerationSubsystem(new DrivetrainAccelerationSubsystem(subsystems.getDriveTrainSubsystem()));
      SmartDashboard.putData("Debug: DrivetrainSub", subsystems.getDriveTrainSubsystem());
      DataLogManager.log("SUCCESS: initializeDrivetrain");

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
      DataLogManager.log("FAIL: initializeDrivetrain");
    }
  }

  /**
   * A method to init the CameraSubsystem
   */
  private void initializeCameraSubsystem(){
    if(InstalledHardware.limelightInstalled) {
      subsystems.setCameraSubsystem(new CameraSubsystem());
      DataLogManager.log("SUCCESS: initializeCamera");
    }
    else {
      DataLogManager.log("FAIL: initializeCamera");
    }
  }

  /**
   * A method to init the LEDSubsystem
   */
  private void initializeLEDSubsystem(){
    if(InstalledHardware.LEDSInstalled){
      subsystems.setLEDSubsystem(new LEDSubsystem(Constants.ledCanID, Constants.ledStripType));
      System.out.println("SUCCESS: initializeLEDS");
    }
    else {
      System.out.println("FAIL: initializeLEDS");
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

      // register the led colors when leds are ready to go - OrangeSolid on note in shooter
      if(subsystems.isLEDSubsystemAvailable()) {
        subsystems.getLedSubsystem().registerStateAction(
        LEDState.OrangeSolid,
        this.subsystems.getFeederSubsystem()::isShooterNoteDetected);
      }
      DataLogManager.log("SUCCESS: FeederSubsystem");
    } else {
      DataLogManager.log("FAIL: FeederSubsystem");
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

      // register the led colors when leds are ready to go - OrangeBlink on note in intake
      if(subsystems.isLEDSubsystemAvailable()) {
        subsystems.getLedSubsystem().registerStateAction(
          LEDState.OrangeBlink,
          this.subsystems.getIntakeSubsystem()::isNoteDetected);
      }
      System.out.println("SUCCESS: IntakeSubsystem");
      DataLogManager.log("SUCCESS: IntakeSubsystem");
    } else {
      DataLogManager.log("FAIL: IntakeSubsystem");
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
      DataLogManager.log("SUCCESS: initializeManualInputInterfaces");
    }
    else {
      DataLogManager.log("FAIL: initializeManualInputInterfaces");
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


      // register the led colors when leds are ready to go - Yellow or Green based on speed of outfeed
      if(subsystems.isLEDSubsystemAvailable()) {
        subsystems.getLedSubsystem().registerStateAction(
          LEDState.Yellow,
          this.subsystems.getShooterOutfeedSubsystem()::isNearSpeed);
        subsystems.getLedSubsystem().registerStateAction(
          LEDState.Green,
          this.subsystems.getShooterOutfeedSubsystem()::isAtSpeed);
      }
      System.out.println("SUCCESS: ShooterOutfeedSubsystem");
      DataLogManager.log("SUCCESS: ShooterOutfeedSubsystem");
    }
    else {
      DataLogManager.log("FAIL: ShooterOutfeedSubsystem");
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
      DataLogManager.log("SUCCESS: ShooterAngleSubsystem");
    }
    else {
      DataLogManager.log("FAIL: ShooterAngleSubsystem");
    }
  }

  /**
   * A method to late binding of default commands
   */
  private void lateBindDefaultCommands() {

    // shooter subsystem default commands
    if(this.subsystems.isShooterOutfeedSubsystemAvailable() && 
    this.subsystems.isManualInputInterfacesAvailable()) {
      // Set up the default command for the shooter.
      this.subsystems.getShooterOutfeedSubsystem().setDefaultCommand(
        new ShootAllStopCommand(this.subsystems.getShooterOutfeedSubsystem()));
    }

    if(this.subsystems.isShooterAngleSubsystemAvailable() && 
    this.subsystems.isManualInputInterfacesAvailable()) {
      // no default command
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
