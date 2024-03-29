// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: ManualInputInterfaces.java
// Intent: Forms a class that grants access to driver controlled inputs.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.control;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;
import frc.robot.commands.DriveToPointCommand;
import frc.robot.commands.IntakeDefaultCommand;
import frc.robot.commands.WristPositionCommand;
import frc.robot.common.ChargedUpGamePiece;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AllStopCommand;
import frc.robot.commands.AutoBalanceStepCommand;
import frc.robot.commands.ButtonPressCommand;
import frc.robot.commands.DriveFinePlacementCommand;
import frc.robot.common.WristPosition;


public class ManualInputInterfaces {

  // sets joystick variables to joysticks
  private CommandXboxController driverController = new CommandXboxController(Constants.portDriverController);
  private CommandXboxController coDriverController = new CommandXboxController(Constants.portCoDriverController);
  private XboxController coDriverControllerForRumbleOnly = new XboxController(Constants.portCoDriverController);

  // subsystems needed for inputs
  private SubsystemCollection subsystemCollection = null;

  // a member to hold the current game piece target - start with cube because
  // manual likely will target cube as start
  private ChargedUpGamePiece coDriverControllerGamePieceTarget = ChargedUpGamePiece.Cube;

  /**
   * The constructor to build this 'manual input' conduit
   */
  public ManualInputInterfaces(SubsystemCollection currentCollection) {
    subsystemCollection = currentCollection;
    displayTargetGamePiece();
  }

  /**
   * A method to return the co driver controller for rumble needs
   * 
   * @return
   */
  public final XboxController getCoDriverController() {
    return coDriverControllerForRumbleOnly;
  }

  /**
   * A method to get the arcade drive X componet being input from humans
   * 
   * @return - a double value associated with the magnitude of the x componet
   */
  public double getInputArcadeDriveX() {
    return driverController.getLeftX();
  }

  /**
   * A method to get the arcade drive X componet being input from humans
   * 
   * @return - a double value associated with the magnitude of the x componet
   */
  public double getInputArcadeDriveY() {
    return driverController.getLeftY();
  }

  /**
   * A method to get the spin drive X componet being input from humans
   * 
   * @return - a double value associated with the magnitude of the x componet
   */
  public double getInputSpinDriveX() {
    return driverController.getRightX();
  }

  /**
   * A method to get the uptake trigger
   * 
   * @return - a double value associated with the magnitude of the right trigger
   *         pull
   */
  public double getInputUptakeTrigger() {
    // use the co drivers right trigger
    double inputValue = 0.0;
    // invert trigger value for cube
    inputValue = coDriverController.getRightTriggerAxis() * -1.0;
    return inputValue;
  }

  /**
   * A method to get the expell trigger
   * 
   * @return - a double value associated with the magnitude of the left trigger
   *         pull
   */
  public double getInputExpellTrigger() {
    // use the co drivers left trigger
    double inputValue = 0.0;
    // invert trigger value for cube
    inputValue = coDriverController.getLeftTriggerAxis();
    return inputValue;
  }

  /**
   * A method to obtain the target game piece
   * 
   * @return the current target game piece
   */
  public ChargedUpGamePiece getTargetGamePiece() {
    return this.coDriverControllerGamePieceTarget;
  }

  /**
   * A method to initialize various commands to the numerous buttons.
   * Need delayed bindings as some subsystems during testing won't always be
   * there.
   */
  public void initializeButtonCommandBindings() {
    // Configure the driver xbox controller bindings
    if (InstalledHardware.driverXboxControllerInstalled) {
      this.bindCommandsToDriverXboxButtons();
    }

    // Configure the co-driver xbox controller bindings
    if (InstalledHardware.coDriverXboxControllerInstalled) {
      this.bindCommandsToCoDriverXboxButtons();
    }
  }

  /**
   * Will attach commands to the Driver XBox buttons
   */
  private void bindCommandsToDriverXboxButtons() {
    if (InstalledHardware.driverXboxControllerInstalled) {

      DrivetrainSubsystem localDrive = subsystemCollection.getDriveTrainSubsystem();

      if (localDrive != null) {

        if (InstalledHardware.applyBasicDriveToPointButtonsToDriverXboxController) {
          this.bindBasicDriveToPointButtonsToDriverXboxController();
        }

        // Back button zeros the gyroscope (as in zero yaw)
        this.driverController.back().onTrue(
            new ParallelCommandGroup(
                new InstantCommand(
                    subsystemCollection.getDriveTrainSubsystem()::zeroGyroscope),
                new ButtonPressCommand(
                    "driverController.back()",
                    "zero gyroscope")));

        // bind the b button to auto balance
        this.driverController.b().onTrue(
            new ParallelCommandGroup(
                new AutoBalanceStepCommand(localDrive),
                new ButtonPressCommand(
                    "driverController.b()",
                    "auto balance")));

        this.driverController.povRight().whileTrue(
            new DriveFinePlacementCommand(
                localDrive,
                -1 * Constants.FinePlacementRotationalVelocity));

        this.driverController.povLeft().whileTrue(
            new DriveFinePlacementCommand(
                localDrive,
                Constants.FinePlacementRotationalVelocity));
      }

      // x button press will stop all
      this.driverController.x().onTrue(
          new ParallelCommandGroup(
              new AllStopCommand(
                  this.subsystemCollection),
              new ButtonPressCommand(
                  "driverController.x()",
                  "!!!!!!!!!!!!!!!!!!!! ALL STOP !!!!!!!!!!!!!!!!!!!!!")));

      if (subsystemCollection.getDriveTrainSubsystem() != null) {
        // left bumper press will decrement power factor
        this.driverController.leftBumper().onTrue(
            new ParallelCommandGroup(
                new InstantCommand(
                    subsystemCollection.getDriveTrainPowerSubsystem()::decrementPowerReductionFactor,
                    subsystemCollection.getDriveTrainPowerSubsystem()),
                new ButtonPressCommand(
                    "driverController.leftBumper()",
                    "decrement power factor")));
        // right bumper press will increment power factor
        this.driverController.rightBumper().onTrue(
            new ParallelCommandGroup(
                new InstantCommand(
                    subsystemCollection.getDriveTrainPowerSubsystem()::incrementPowerReductionFactor,
                    subsystemCollection.getDriveTrainPowerSubsystem()),
                new ButtonPressCommand(
                    "driverController.rightBumper()",
                    "increment power factor")));
        // right trigger press will put drivetrain in immoveable stance
        // DO NOT require drivetrainSubsystem here. We need the default command to
        // continue to decel the robot.
        this.driverController.rightTrigger().onTrue(
            new ParallelCommandGroup(
                new InstantCommand(
                    () -> subsystemCollection.getDriveTrainSubsystem()
                        .setSwerveDriveMode(SwerveDriveMode.IMMOVABLE_STANCE)),
                new ButtonPressCommand(
                    "driverController.rightTrigger()",
                    "immoveable stance")));
        // right trigger de-press will put drivetrain in normal drive mode
        this.driverController.rightTrigger().onFalse(
            new ParallelCommandGroup(
                new InstantCommand(
                    () -> subsystemCollection.getDriveTrainSubsystem()
                        .setSwerveDriveMode(SwerveDriveMode.NORMAL_DRIVING)),
                new ButtonPressCommand(
                    "driverController.rightTrigger()",
                    "normal driving")));
        // left trigger press will ramp down drivetrain to reduced speed mode
        this.driverController.leftTrigger().onTrue(
            new ParallelCommandGroup(
                new InstantCommand(subsystemCollection.getDriveTrainPowerSubsystem()::setReducedPowerReductionFactor,
                    subsystemCollection.getDriveTrainPowerSubsystem()),
                new ButtonPressCommand(
                    "driverController.leftTrigger()",
                    "ramp down to reduced speed")));
        // left trigger de-press will ramp up drivetrain to max speed
        this.driverController.leftTrigger().onFalse(
            new ParallelCommandGroup(
                new InstantCommand(subsystemCollection.getDriveTrainPowerSubsystem()::resetPowerReductionFactor,
                    subsystemCollection.getDriveTrainPowerSubsystem()),
                new ButtonPressCommand(
                    "driverController.leftTrigger()",
                    "ramp up to default speed")));
      }
    }
  }

  /**
   * A method that will bind buttons for basic drive to point commands to driver
   * controller
   */
  private void bindBasicDriveToPointButtonsToDriverXboxController() {
    // basic x translation negative one meter
    this.driverController.a().onTrue(
        new ParallelCommandGroup(
            new DriveToPointCommand(
                this.subsystemCollection.getDriveTrainSubsystem(),
                this.getTargetPosition(-1.0, 0.0, 0.0)),
            new ButtonPressCommand(
                "driverController.a()",
                "-1.0m X translation DriveToPointCommand"))
            .withTimeout(10.0));
    // basic x translation positive one meter
    this.driverController.y().onTrue(
        new ParallelCommandGroup(
            new DriveToPointCommand(
                this.subsystemCollection.getDriveTrainSubsystem(),
                this.getTargetPosition(1.0, 0.0, 0.0)),
            new ButtonPressCommand(
                "driverController.y()",
                "1.0m X translation DriveToPointCommand"))
            .withTimeout(10.0));
    // basic y translation negative one meter
    this.driverController.b().onTrue(
        new ParallelCommandGroup(
            new DriveToPointCommand(
                this.subsystemCollection.getDriveTrainSubsystem(),
                this.getTargetPosition(0.0, -1.0, 0.0)),
            new ButtonPressCommand(
                "driverController.b()",
                "-1.0m y translation DriveToPointCommand"))
            .withTimeout(10.0));
    // basic y translation positive one meter
    this.driverController.x().onTrue(
        new ParallelCommandGroup(
            new DriveToPointCommand(
                this.subsystemCollection.getDriveTrainSubsystem(),
                this.getTargetPosition(0.0, 1.0, 0.0)),
            new ButtonPressCommand(
                "driverController.x()",
                "1.0m y translation DriveToPointCommand"))
            .withTimeout(10.0));
    // basic rotation of 90 degrees
    this.driverController.leftBumper().onTrue(
        new ParallelCommandGroup(
            new DriveToPointCommand(
                this.subsystemCollection.getDriveTrainSubsystem(),
                this.getTargetPosition(0.0, 0.0, 90.0)),
            new ButtonPressCommand(
                "driverController.leftBumper()",
                "90 degree rotation DriveToPointCommand"))
            .withTimeout(10.0));
    // basic rotation of -90 degrees
    this.driverController.rightBumper().onTrue(
        new ParallelCommandGroup(
            new DriveToPointCommand(
                this.subsystemCollection.getDriveTrainSubsystem(),
                this.getTargetPosition(0.0, 0.0, -90.0)),
            new ButtonPressCommand(
                "driverController.rightBumper()",
                "-90 degree rotation DriveToPointCommand"))
            .withTimeout(10.0));
  }

  /**
   * displays target game piece on smartdashboard
   */
  private void displayTargetGamePiece() {
    SmartDashboard.putBoolean("ConeMode", this.coDriverControllerGamePieceTarget == ChargedUpGamePiece.Cone);
    SmartDashboard.putBoolean("CubeMode", this.coDriverControllerGamePieceTarget == ChargedUpGamePiece.Cube);
  }

  /**
   * A method to do the transformation of current robot position to another
   * position
   * 
   * @param xTranslation
   * @param yTranslation
   * @param rotationDegrees
   * @return
   */
  private Pose2d getTargetPosition(double xTranslation, double yTranslation, double rotationDegrees) {
    Pose2d startPos = this.subsystemCollection.getDriveTrainSubsystem().getRobotPosition();
    Translation2d theTranslation = new Translation2d(xTranslation, yTranslation);
    Rotation2d theRotation = Rotation2d.fromDegrees(rotationDegrees);
    Transform2d theTransform = new Transform2d(theTranslation, theRotation);
    return startPos.transformBy(theTransform);
  }

  /**
   * Will attach commands to the Co Driver XBox buttons
   */
  private void bindCommandsToCoDriverXboxButtons() {
    if (InstalledHardware.coDriverXboxControllerInstalled) {
      // left trigger variable press will intake
      // right trigger variable press will expell

     
    //Intake Bumpers
    // Start cargo uptake when the A button is pressed:
    this.coDriverController.leftBumper().whileTrue(
        new ParallelCommandGroup(
            new IntakeDefaultCommand(subsystemCollection.getWristSubsystem(),
                                     subsystemCollection.getIntakeSubsystem(), 
                                     () -> 1.0, // Assuming full power uptake
                                     () -> 0.0), // No expelling
            new ButtonPressCommand("coDriverController.LeftBumper()", "Start cargo uptake"))
    );

    

    // Start cargo expulsion when the B button is pressed:
    this.coDriverController.rightBumper().whileTrue(
        new ParallelCommandGroup(
            new IntakeDefaultCommand(subsystemCollection.getWristSubsystem(),
                                     subsystemCollection.getIntakeSubsystem(), 
                                     () -> 0.0, // No uptake
                                     () -> 0.3), // Assuming full power expelling
            new ButtonPressCommand("coDriverController.rightBumper()", "Start cargo expulsion"))
    );


    // Ponder
    this.coDriverController.povDown().whileTrue(
        new ParallelCommandGroup(
            new IntakeDefaultCommand(subsystemCollection.getWristSubsystem(),
                                     subsystemCollection.getIntakeSubsystem(), 
                                     () -> 0.0, // No uptake
                                     () -> 0.9, // Assuming ponder-speed expelling
                                     true), 
            new ButtonPressCommand("coDriverController.rightBumper()", "Ponder"))
    );

    //Wrist Positions
    // Set wrist to pickup position when the X button is pressed:
    this.coDriverController.a().onTrue(
        new ParallelCommandGroup(
            new WristPositionCommand(subsystemCollection.getWristSubsystem(), WristPosition.PickUp),
            new ButtonPressCommand("driverController.X()", "Set wrist to pickup (0st) position"))
            .withTimeout(5.0)
    );

    this.coDriverController.x().onTrue(
        new ParallelCommandGroup(
            new WristPositionCommand(subsystemCollection.getWristSubsystem(), WristPosition.PositionOne),
            new ButtonPressCommand("driverController.X()", "Set wrist to 1st position"))
            .withTimeout(5.0)
    );

    this.coDriverController.y().onTrue(
        new ParallelCommandGroup(
            new WristPositionCommand(subsystemCollection.getWristSubsystem(), WristPosition.PositionTwo),
            new ButtonPressCommand("driverController.X()", "Set wrist to 2nd position"))
            .withTimeout(5.0)
    );

    this.coDriverController.b().onTrue(
        new ParallelCommandGroup(
            new WristPositionCommand(subsystemCollection.getWristSubsystem(), WristPosition.PositionThree),
            new ButtonPressCommand("driverController.X()", "Set wrist to 3rd position"))
            .withTimeout(5.0)
    );


    }
  }

}
