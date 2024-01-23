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
import frc.robot.common.TestTrajectories;
import frc.robot.common.SwerveTrajectoryConfig;
import frc.robot.commands.*;
import frc.robot.control.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
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

    // init the input system
    this.initializeManualInputInterfaces();

    // init wrist system
    this.initializeWristSubsystem();

    // arm and picker later
    this.initializeIntakeSubsystem();

    // calculate and update the current position of the robot
    this.calculateAndUpdateRobotPosition();

    // Configure the button bindings
    System.out.println(">>>> Initializing button bindings.");
    this.subsystems.getManualInputInterfaces().initializeButtonCommandBindings();
    System.out.println(">>>> Finished initializing button bindings.");

    // zero the wrist
    new ZeroWristEncoderCommand(this.subsystems.getWristSubsystem());
    System.out.println(">>>> Finished Initing Wrist");

    this.initializeDebugDashboard();
    this.autonomousChooser = new AutonomousChooser(subsystems);
    // TODO refactor the commands in ManualInputInterfaces:
    // bindBasicDriveToPointButtonsToDriverXboxController and
    // bindDriveTrajectoryButtonsToDriverXboxController
    // to instead be commands on the shuffleboard like this:
    // SmartDashboard.putData("Example Command", exampleCommand);

    private final TestTrajectories testTrajectories = new TestTrajectories(subsystems.getDriveTrainSubsystem().getTrajectoryConfig());

    // Command to drive the chassis for zeroing the swerve modules.
    SmartDashboard.putData("Drive Forward Robot Centric",
        new DriveTimeCommand(this.subsystems.getDriveTrainSubsystem(),
            new ChassisSpeeds(0.6, 0.0, 0.0), 3.0));

    SmartDashboard.putData("Drive Forward Trajectory",
      new DriveTrajectoryCommand(subsystems.getDriveTrainSubsystem(), testTrajectories.traverseSimpleForward));
    
    SmartDashboard.putData("Drive ZigZag Trajectory",
      new DriveTrajectoryCommand(subsystems.getDriveTrainSubsystem(), testTrajectories.traverseZigZag));

    SmartDashboard.putData("Drive Turn90 Trajectory",
      new DriveTrajectoryCommand(subsystems.getDriveTrainSubsystem(), testTrajectories.turn90));

    SmartDashboard.putData("Drive Forward with rotation",
        new DriveTimeCommand(this.subsystems.getDriveTrainSubsystem(),
            new ChassisSpeeds(0.6, 0.0, 0.2), 3.0));
    SmartDashboard.putData("Print NavX State",
        new InstantCommand(this.subsystems.getDriveTrainSubsystem()::printState));

    SmartDashboard.putData("Zero Wrist", new ZeroWristEncoderCommand(this.subsystems.getWristSubsystem()));
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
   * A method called at the beginning of teleop
  */
  public void teleopInit() {
  }

  /**
   * A method to init the input interfaces
   */
  private void initializeManualInputInterfaces() {
    // note: in this case it is safe to build the interfaces if only one of the
    // controllers is present
    // because button binding assignment code checks that each is installed later
    // (see: initializeButtonCommandBindings)
    if (InstalledHardware.driverXboxControllerInstalled) {
      subsystems.setManualInputInterfaces(new ManualInputInterfaces(subsystems));
      System.out.println("SUCCESS: initializeManualInputInterfaces");
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
        () -> -modifyAxisSquare(subsystems.getManualInputInterfaces().getInputArcadeDriveY())
            * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxisSquare(subsystems.getManualInputInterfaces().getInputArcadeDriveX())
            * DrivetrainSubsystem.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxisSquare(subsystems.getManualInputInterfaces().getInputSpinDriveX())
            * DrivetrainSubsystem.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));
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
   * A method to init the intake
   */
  private void initializeIntakeSubsystem() {
    if (InstalledHardware.intakeInstalled) {
      subsystems.setIntakeSubsystem(new IntakeSubsystem());
      subsystems.getIntakeSubsystem().setDefaultCommand(new IntakeDefaultCommand(
          subsystems.getWristSubsystem(),
          subsystems.getIntakeSubsystem(),
          () -> modifyAxisSquare(subsystems.getManualInputInterfaces().getInputUptakeTrigger()),
          () -> modifyAxisSquare(subsystems.getManualInputInterfaces().getInputExpellTrigger())));
      SmartDashboard.putData("Debug: IntakeSub", subsystems.getIntakeSubsystem());
      System.out.println("SUCCESS: initializeIntake");
    }
  }

  /**
   * A method init the wrist
   */
  private void initializeWristSubsystem() {
    if (InstalledHardware.wristInstalled) {
      subsystems.setWristSubsystem(new WristSubsystem());
    }
  }

  /**
   * A method to calculate the initial position of the robot
   */
  private void calculateAndUpdateRobotPosition() {
    Pose2d initialRobotPosition = new Pose2d();
    // TODO - need to implement this when we have vision
    // 1. find the April tag that is closest
    // 2. estimate the robot centroid location
    // 3. find other April tags ...
    // 4. apply some smoothing
    if (subsystems.getDriveTrainSubsystem() != null) {
      subsystems.getDriveTrainSubsystem().setRobotPosition(initialRobotPosition);
      System.out.println(">>>> Initialized Robot Position. ");
    }
  }

  private static double deadband(double value, double deadband) {
    if (Math.abs(value) > deadband) {
      if (value > 0.0) {
        return (value - deadband) / (1.0 - deadband);
      } else {
        return (value + deadband) / (1.0 - deadband);
      }
    } else {
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