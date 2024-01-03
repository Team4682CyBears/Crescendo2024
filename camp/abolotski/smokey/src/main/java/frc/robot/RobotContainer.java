// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: RobotContainer.java
// Intent: Forms key glue class to land subsystems and input devices that will themselves create commands applied to the robot.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ArmToLocationCommand;
import frc.robot.commands.ArmToReferencePositionCommand;
import frc.robot.commands.DefaultArmCommand;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.DriveTimeCommand;
import frc.robot.commands.EveryBotPickerDefaultCommand;
import frc.robot.commands.EveryBotPickerOverCurrentCommand;
import frc.robot.commands.AllignWithTag;
import frc.robot.commands.RumbleCommand;
import frc.robot.commands.ArmToLocationCommand.ArmLocation;
import frc.robot.control.AutonomousChooser;
import frc.robot.control.InstalledHardware;
import frc.robot.control.ManualInputInterfaces;
import frc.robot.control.SubsystemCollection;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.DrivetrainPowerSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.EveryBotPickerSubsystem;
import frc.robot.subsystems.PickerSubsystem;
import frc.robot.subsystems.PowerDistributionPanelWatcherSubsystem;
import frc.robot.subsystems.StabilizerSubsystem;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.common.PortSpy;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private SubsystemCollection subsystems = new SubsystemCollection();
  private final AutonomousChooser autonomousChooser; 

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {

    // init the pdp watcher
    this.initializePowerDistributionPanelWatcherSubsystem();

    //init the camera (before the drivetrain)
    this.initializeCameraSubsystem();

    // init the various subsystems
    this.initializeDrivetrainSubsystem();
    this.initializeStablizerSubsystem();

    // init the input system 
    this.initializeManualInputInterfaces();

    // arm and picker later
    this.initializeArmSubsystem();
    this.initializeEveryBotPickerSubsystem();
    this.initializePickerSubsystem();

    // calculate and update the current position of the robot
    this.calculateAndUpdateRobotPosition();

    // Configure the button bindings
    System.out.println(">>>> Initializing button bindings.");
    this.subsystems.getManualInputInterfaces().initializeButtonCommandBindings();
    System.out.println(">>>> Finished initializing button bindings.");

    this.initializeDebugDashboard();
    this.autonomousChooser = new AutonomousChooser(subsystems);
    // TODO refactor the commands in ManualInputInterfaces:
    // bindBasicDriveToPointButtonsToDriverXboxController and bindDriveTrajectoryButtonsToDriverXboxController 
    // to instead be commands on the shuffleboard like this:
    // SmartDashboard.putData("Example Command", exampleCommand);
    SmartDashboard.putData("Allign With Tag", new AllignWithTag(1, this.subsystems.getDriveTrainSubsystem()));

    // Command to drive the chassis for zeroing the swerve modules.
    SmartDashboard.putData("Drive Forward Robot Centric", 
      new DriveTimeCommand(this.subsystems.getDriveTrainSubsystem(), 
      new ChassisSpeeds(0.6, 0.0, 0.0), 3.0));
    SmartDashboard.putData("Drive Forward with rotation", 
      new DriveTimeCommand(this.subsystems.getDriveTrainSubsystem(), 
      new ChassisSpeeds(0.6, 0.0, 0.2), 3.0));
    SmartDashboard.putData("Print NavX State", 
      new InstantCommand(this.subsystems.getDriveTrainSubsystem()::printState));
  }
 
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return autonomousChooser.getCommand();
  }


  /**
   * 
   */
  public void teleopInit()
  {
    // NOTE this is enabled in teleop, because when enabled in Auto, it usurps the rest of the auto routine.  
    if(this.subsystems.getEveryBotPickerSubsystem() != null){
          // add a watcher for overcurrent on the 
          EveryBotPickerOverCurrentCommand ebCmd = new EveryBotPickerOverCurrentCommand(
            subsystems.getEveryBotPickerSubsystem(), Constants.overcurrentRumbleTimeSeconds);
          RumbleCommand rc = new RumbleCommand(
            this.subsystems.getManualInputInterfaces().getCoDriverController(),
            Constants.overcurrentRumbleTimeSeconds);
          
          // TODO - PDP watcher code needs testing and fine tuning
          subsystems.getPowerDistributionPanelWatcherSubsystem().add(
            new PortSpy(
              Constants.EveryBotMotorPdpPortId,
              Constants.EveryBotMotorMaximuCurrentAmps,
              new SequentialCommandGroup( ebCmd, rc),
              "EveryBotMotorOvercurrentProtection"
            )
          );
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
      subsystems.setDriveTrainPowerSubsystem(new DrivetrainPowerSubsystem(subsystems.getDriveTrainSubsystem()));
      SmartDashboard.putData("Debug: DrivetrainSub", subsystems.getDriveTrainSubsystem());
      System.out.println("SUCCESS: initializeDrivetrain");

      // Set up the default command for the drivetrain.
      // The controls are for field-oriented driving:
      // Left stick Y axis -> forward and backwards movement
      // Left stick X axis -> left and right movement
      // Right stick X axis -> rotation
      subsystems.getDriveTrainSubsystem().setDefaultCommand(new DefaultDriveCommand(
        subsystems.getDriveTrainSubsystem(),
        () -> -modifyAxisSquare(subsystems.getManualInputInterfaces().getInputArcadeDriveY()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxisSquare(subsystems.getManualInputInterfaces().getInputArcadeDriveX()) * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxisSquare(subsystems.getManualInputInterfaces().getInputSpinDriveX()) * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
      ));
    }
    else {
      System.out.println("FAIL: initializeDrivetrain");
    }
  }
  
  /**
   * A method to init the arm
   */
  private void initializeArmSubsystem() {
    if(InstalledHardware.verticalArmMotorInstalled && 
      InstalledHardware.horizontalArmMotorInstalled) {
      // The robot's subsystems and commands are defined here...
      subsystems.setArmSubsystem(new ArmSubsystem());
      SmartDashboard.putData("Debug: ArmSub", subsystems.getArmSubsystem());
      System.out.println("SUCCESS: initializeArm");

      // Set up the default command for the arm.
      // Left stick X axis -> horizontal arm in / out movement
      // Left stick Y axis -> vertical arm in / out movement
      subsystems.getArmSubsystem().setDefaultCommand(new DefaultArmCommand(
        subsystems.getArmSubsystem(),
        () -> subsystems.getManualInputInterfaces().getInputArcadeArmY(),
        () -> subsystems.getManualInputInterfaces().getInputArcadeArmZ()
      ));
    }
    else {
      System.out.println("FAIL: initializeArm");
    }
  }

  /**
   * A mothod to init the Limelight
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
   * A method to init the picker
   */
  private void initializePickerSubsystem() {
    if(InstalledHardware.pickerPneumaticsInstalled) {
      subsystems.setPickerSubsystem(new PickerSubsystem());
      SmartDashboard.putData("Debug: PickerSub", subsystems.getPickerSubsystem());
      System.out.println("SUCCESS: initializePicker");
    }
    else {
      System.out.println("FAIL: initializePicker");
    }
  }

  /**
   * A method to init the every bot picker
   */
  private void initializeEveryBotPickerSubsystem() {
    if(InstalledHardware.everyBotPickerInstalled) {
      subsystems.setEveryBotPickerSubsystem(new EveryBotPickerSubsystem());
      subsystems.getEveryBotPickerSubsystem().setDefaultCommand(new EveryBotPickerDefaultCommand(
        subsystems.getEveryBotPickerSubsystem(),
        () -> modifyAxisSquare(subsystems.getManualInputInterfaces().getInputEveryBotUptakeTrigger()),
        () -> modifyAxisSquare(subsystems.getManualInputInterfaces().getInputEveryBotExpellTrigger())
      ));
      SmartDashboard.putData("Debug: EveryBotSub", subsystems.getEveryBotPickerSubsystem());
      System.out.println("SUCCESS: initializeEveryBotPicker");
    }
    else {
      System.out.println("FAIL: initializeEveryBotPicker");
    }
  }

  /**
   * A method to init the stablizer
   */
  private void initializeStablizerSubsystem() {
    if(InstalledHardware.stablizerPneumaticsInstalled) {
      subsystems.setStabilizerSubsystem(new StabilizerSubsystem());
      SmartDashboard.putData("Debug: StabilizerSub", subsystems.getStabilizerSubsystem());
      System.out.println("SUCCESS: initializeStablizer");
    }
    else {
      System.out.println("FAIL: initializeStablizer");
    }
  }

  /**
   * A method to calculate the initial position of the robot
   */
  private void calculateAndUpdateRobotPosition() {
    // TODO - do this right!
    Pose2d initialRobotPosition = new Pose2d();
    // TODO - need to implement this when we have vision
    // 1. find the April tag that is closest
    // 2. estimate the robot centroid location
    // 3. find other April tags ...
    // 4. apply some smoothing
    if(subsystems.getDriveTrainSubsystem() != null) {
      subsystems.getDriveTrainSubsystem().setRobotPosition(initialRobotPosition);
      System.out.println(">>>> Initialized Robot Position. ");
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

  private static double modifyAxisLinear(double value) {
    // Deadband
    value = deadband(value, 0.05);

    // Joystick input exponent
    value = Math.copySign(value, value);

    return value;
  }
}