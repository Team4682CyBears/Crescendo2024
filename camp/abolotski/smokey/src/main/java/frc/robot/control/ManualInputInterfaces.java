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
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.*;
import frc.robot.commands.DriveToPointCommand;
import frc.robot.commands.DriveTrajectoryCommand;
import frc.robot.commands.ManipulatePickerCommand;
import frc.robot.commands.RumbleCommand;
import frc.robot.commands.ArmToLocationCommand.ArmLocation;
import frc.robot.common.ChargedUpGamePiece;
import frc.robot.common.TestTrajectories;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AllStopCommand;
import frc.robot.commands.ArmToLocationCommand;
import frc.robot.commands.AutoBalanceStepCommand;
import frc.robot.commands.ButtonPressCommand;
import frc.robot.commands.DriveFinePlacementCommand;
import frc.robot.commands.DriveRampDownSpeedCommand;
import frc.robot.commands.DriveRampUpSpeedCommand;

public class ManualInputInterfaces {

  // sets joystick variables to joysticks
  private CommandXboxController driverController = new CommandXboxController(Constants.portDriverController); 
  private CommandXboxController coDriverController = new CommandXboxController(Constants.portCoDriverController);
  private XboxController coDriverControllerForRumbleOnly = new XboxController(Constants.portCoDriverController);

  // subsystems needed for inputs
  private SubsystemCollection subsystemCollection = null;

  // a member to hold the current game piece target - start with cube because manual likely will target cube as start
  private ChargedUpGamePiece coDriverControllerGamePieceTarget = ChargedUpGamePiece.Cube;

  /**
   * The constructor to build this 'manual input' conduit
   */
  public ManualInputInterfaces(SubsystemCollection currentCollection){
    subsystemCollection = currentCollection;
    displayTargetGamePiece();
  }

  /**
   * A method to return the co driver controller for rumble needs
   * @return
   */
  public final XboxController getCoDriverController() {
    return coDriverControllerForRumbleOnly;
  }

  /**
   * A method to get the arcade drive X componet being input from humans
   * @return - a double value associated with the magnitude of the x componet
   */
  public double getInputArcadeDriveX(){
    return driverController.getLeftX();
  }

  /**
   * A method to get the arcade drive X componet being input from humans
   * @return - a double value associated with the magnitude of the x componet
   */
  public double getInputArcadeDriveY(){
    return driverController.getLeftY();
  }

  /**
   * A method to get the spin drive X componet being input from humans
   * @return - a double value associated with the magnitude of the x componet
   */
  public double getInputSpinDriveX(){
    return driverController.getRightX();
  }

  /**
   * A method to get the arcade arm Y componet being input from humans
   * @return - a double value associated with the magnitude of the Y componet
   */
  public double getInputArcadeArmY()
  {
    // use the co drivers right X to represent the horizontal movement
    // and multiply by -1.0 as xbox reports values flipped
    return -1.0 * coDriverController.getLeftY();
  }

  /**
   * A method to get the arcade arm Z componet being input from humans
   * @return - a double value associated with the magnitude of the Z componet
   */
  public double getInputArcadeArmZ()
  {
    // use the co drivers right Z to represent the vertical movement
    // and multiply by -1.0 as xbox reports values flipped
    return -1.0 * coDriverController.getRightY();
  }

  /**
   * A method to get the every bot uptake trigger
   * @return - a double value associated with the magnitude of the right trigger pull
   */
  public double getInputEveryBotUptakeTrigger()
  {
    // use the co drivers right trigger
    double inputValue = 0.0;
    if(this.coDriverControllerGamePieceTarget == ChargedUpGamePiece.Cone) {
      inputValue = coDriverController.getRightTriggerAxis();
    }
    else if (this.coDriverControllerGamePieceTarget == ChargedUpGamePiece.Cube) {
      inputValue = coDriverController.getRightTriggerAxis() * -1.0;
    }
    return inputValue;
  }

  /**
   * A method to get the every bot expell trigger
   * @return - a double value associated with the magnitude of the left trigger pull
   */
  public double getInputEveryBotExpellTrigger()
  {
    // use the co drivers left trigger
    double inputValue = 0.0;
    if(this.coDriverControllerGamePieceTarget == ChargedUpGamePiece.Cone) {
      inputValue = coDriverController.getLeftTriggerAxis() * -1.0;
    }
    else if (this.coDriverControllerGamePieceTarget == ChargedUpGamePiece.Cube) {
      inputValue = coDriverController.getLeftTriggerAxis();
    }
    return inputValue;
  }

  /**
   * A method to obtain the target game piece
   * @return the current target game piece
   */
  public ChargedUpGamePiece getTargetGamePiece() {
    return this.coDriverControllerGamePieceTarget;
  }

  /**
   * A method to set the target game piece as Cone
   */
  public void setTargetGamePieceAsCone() {
    this.coDriverControllerGamePieceTarget = ChargedUpGamePiece.Cone;
    displayTargetGamePiece();
  }
  /**
   * A method to set the target game piece as Cube
   */
  public void setTargetGamePieceAsCube() {
    this.coDriverControllerGamePieceTarget = ChargedUpGamePiece.Cube;
    displayTargetGamePiece();
  }

  /**
   * A method to initialize various commands to the numerous buttons.
   * Need delayed bindings as some subsystems during testing won't always be there.
   */
  public void initializeButtonCommandBindings()
  {
    // Configure the driver xbox controller bindings
    if(InstalledHardware.driverXboxControllerInstalled){
      this.bindCommandsToDriverXboxButtons();
    }

    // Configure the co-driver xbox controller bindings
    if(InstalledHardware.coDriverXboxControllerInstalled){
      this.bindCommandsToCoDriverXboxButtons();
    }
  }

  /**
   * Will attach commands to the Driver XBox buttons 
   */
  private void bindCommandsToDriverXboxButtons(){
    if(InstalledHardware.driverXboxControllerInstalled){
      
      DrivetrainSubsystem localDrive = subsystemCollection.getDriveTrainSubsystem();

      if(localDrive != null){

        if(InstalledHardware.applyBasicDriveToPointButtonsToDriverXboxController){
          this.bindBasicDriveToPointButtonsToDriverXboxController();
        }
        if(InstalledHardware.applyDriveTrajectoryButtonsToDriverXboxController){
          this.bindDriveTrajectoryButtonsToDriverXboxController();
        }

        // Back button zeros the gyroscope (as in zero yaw)
        this.driverController.back().onTrue(
          new ParallelCommandGroup(
            new InstantCommand(
              subsystemCollection.getDriveTrainSubsystem()::zeroGyroscope),
            new ButtonPressCommand(
              "driverController.back()",
              "zero gyroscope")
            )
          );

        // bind the b button to auto balance
        this.driverController.b().onTrue(
          new ParallelCommandGroup(
            new AutoBalanceStepCommand(localDrive),
            new ButtonPressCommand(
              "driverController.b()",
              "auto balance")
            )
          );

          this.driverController.povRight().whileTrue(
            new DriveFinePlacementCommand(
              localDrive, 
              -1 * Constants.FinePlacementRotationalVelocity
              )
            ); 
          
          this.driverController.povLeft().whileTrue(
            new DriveFinePlacementCommand(
              localDrive, 
              Constants.FinePlacementRotationalVelocity
              )
            ); 
    }

      // x button press will stop all      
      this.driverController.x().onTrue(
        new ParallelCommandGroup(
          new AllStopCommand(
            this.subsystemCollection),
          new ButtonPressCommand(
            "driverController.x()",
            "!!!!!!!!!!!!!!!!!!!! ALL STOP !!!!!!!!!!!!!!!!!!!!!")
          )
      );

      if(subsystemCollection.getDriveTrainSubsystem() != null){
        // left bumper press will decrement power factor  
        this.driverController.leftBumper().onTrue(
          new ParallelCommandGroup(
            new InstantCommand(
              subsystemCollection.getDriveTrainPowerSubsystem()::decrementPowerReductionFactor,
              subsystemCollection.getDriveTrainPowerSubsystem()),
            new ButtonPressCommand(
              "driverController.leftBumper()",
              "decrement power factor")
            )
          );
        // right bumper press will increment power factor  
        this.driverController.rightBumper().onTrue(
          new ParallelCommandGroup(
            new InstantCommand(
              subsystemCollection.getDriveTrainPowerSubsystem()::incrementPowerReductionFactor,
              subsystemCollection.getDriveTrainPowerSubsystem()),
            new ButtonPressCommand(
              "driverController.rightBumper()",
              "increment power factor")
            )
          );
        // right trigger press will put drivetrain in immoveable stance
        // DO NOT require drivetrainSubsystem here.  We need the default command to continue to decel the robot.    
        this.driverController.rightTrigger().onTrue(
          new ParallelCommandGroup(
            new InstantCommand(
              () -> subsystemCollection.getDriveTrainSubsystem().setSwerveDriveMode(SwerveDriveMode.IMMOVABLE_STANCE)
            ),
            new ButtonPressCommand(
            "driverController.rightTrigger()",
            "immoveable stance")
          )
        );
        // right trigger de-press will put drivetrain in normal drive mode  
        this.driverController.rightTrigger().onFalse(
          new ParallelCommandGroup(
            new InstantCommand(
              () -> subsystemCollection.getDriveTrainSubsystem().setSwerveDriveMode(SwerveDriveMode.NORMAL_DRIVING)
            ),
            new ButtonPressCommand(
            "driverController.rightTrigger()",
            "normal driving")
          )
        );
        // left trigger press will ramp down drivetrain to reduced speed mode 
        this.driverController.leftTrigger().onTrue(
          new ParallelCommandGroup(
            new InstantCommand(subsystemCollection.getDriveTrainPowerSubsystem()::setReducedPowerReductionFactor,
            subsystemCollection.getDriveTrainPowerSubsystem()),
            new ButtonPressCommand(
            "driverController.leftTrigger()",
            "ramp down to reduced speed")
          )
        );
        // left trigger de-press will ramp up drivetrain to max speed
        this.driverController.leftTrigger().onFalse(
          new ParallelCommandGroup(
            new InstantCommand(subsystemCollection.getDriveTrainPowerSubsystem()::resetPowerReductionFactor,
            subsystemCollection.getDriveTrainPowerSubsystem()),
            new ButtonPressCommand(
            "driverController.leftTrigger()",
            "ramp up to default speed")
          )
        );
      }

      if(subsystemCollection.getStabilizerSubsystem() != null) {
        // dpad down press will deploy the stablizer      
        this.driverController.povDown().onTrue(
          new ParallelCommandGroup(
            new InstantCommand(
              subsystemCollection.getStabilizerSubsystem()::deployPosition),
            new ButtonPressCommand(
              "driverController.povDown()",
              "deploy stablizer")
            )
          );
        // dpad up press will retract the stablizer
        this.driverController.povUp().onTrue(
          new ParallelCommandGroup(
            new InstantCommand(
              subsystemCollection.getStabilizerSubsystem()::retractPosition),
            new ButtonPressCommand(
              "driverController.povUp()",
              "retract stablizer")
            )
          );
      }
      
    }
  }

  /**
   * A method that will bind buttons for basic drive to point commands to driver controller
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
          "-1.0m X translation DriveToPointCommand")).withTimeout(10.0)
    );
    // basic x translation positive one meter
    this.driverController.y().onTrue(
      new ParallelCommandGroup(
        new DriveToPointCommand(
          this.subsystemCollection.getDriveTrainSubsystem(),
          this.getTargetPosition(1.0, 0.0, 0.0)),
        new ButtonPressCommand(
          "driverController.y()",
          "1.0m X translation DriveToPointCommand")).withTimeout(10.0)
    );      
    // basic y translation negative one meter
    this.driverController.b().onTrue(
      new ParallelCommandGroup(
        new DriveToPointCommand(
          this.subsystemCollection.getDriveTrainSubsystem(),
          this.getTargetPosition(0.0, -1.0, 0.0)),
        new ButtonPressCommand(
          "driverController.b()",
          "-1.0m y translation DriveToPointCommand")).withTimeout(10.0)
    );
    // basic y translation positive one meter
    this.driverController.x().onTrue(
      new ParallelCommandGroup(
        new DriveToPointCommand(
          this.subsystemCollection.getDriveTrainSubsystem(),
          this.getTargetPosition(0.0, 1.0, 0.0)),
        new ButtonPressCommand(
          "driverController.x()",
          "1.0m y translation DriveToPointCommand")).withTimeout(10.0)
    );
    // basic rotation of 90 degrees
    this.driverController.leftBumper().onTrue(
      new ParallelCommandGroup(
        new DriveToPointCommand(
          this.subsystemCollection.getDriveTrainSubsystem(),
          this.getTargetPosition(0.0, 0.0, 90.0)),
        new ButtonPressCommand(
          "driverController.leftBumper()",
          "90 degree rotation DriveToPointCommand")).withTimeout(10.0)
    );
    // basic rotation of -90 degrees
    this.driverController.rightBumper().onTrue(
      new ParallelCommandGroup(
        new DriveToPointCommand(
          this.subsystemCollection.getDriveTrainSubsystem(),
          this.getTargetPosition(0.0, 0.0, -90.0)),
        new ButtonPressCommand(
          "driverController.rightBumper()",
          "-90 degree rotation DriveToPointCommand")).withTimeout(10.0)
    );
  }

  /**
   * A method that will bind buttons to have the robot flow through various trajectories
   */
  private void bindDriveTrajectoryButtonsToDriverXboxController() {
    // trajectories
    TestTrajectories testTrajectories = new TestTrajectories(subsystemCollection.getDriveTrainSubsystem().getTrajectoryConfig());

    // traverse forward arc trajectory
    this.driverController.a().onTrue(
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new InstantCommand(subsystemCollection.getDriveTrainSubsystem()::zeroRobotPosition),
          new DriveTrajectoryCommand(
            this.subsystemCollection.getDriveTrainSubsystem(),
            testTrajectories.traverseForwardArc)),
        new ButtonPressCommand(
          "driverController.a()",
          "testTrajectories.traverseForwardArc")).withTimeout(10.0)
    );
    // traverse backward arc trajectory
    this.driverController.b().onTrue(
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new InstantCommand(() -> subsystemCollection.getDriveTrainSubsystem()
          .setRobotPosition(testTrajectories.traverseBackwardArcStartPosition)),
          new DriveTrajectoryCommand(
            this.subsystemCollection.getDriveTrainSubsystem(),
            testTrajectories.traverseBackwardArc)),
        new ButtonPressCommand(
          "driverController.b()",
          "testTrajectories.traverseBackwardArc")).withTimeout(10.0)
    );
    // traverse simple forward trajectory
    this.driverController.x().onTrue(
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new InstantCommand(subsystemCollection.getDriveTrainSubsystem()::zeroRobotPosition),
          new DriveTrajectoryCommand(
            this.subsystemCollection.getDriveTrainSubsystem(),
            testTrajectories.traverseSimpleForward)),
        new ButtonPressCommand(
          "driverController.x()",
          "testTrajectories.traverseSimpleForward")).withTimeout(10.0)
    );
    // traverse simple left trajectory
    this.driverController.y().onTrue(
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new InstantCommand(subsystemCollection.getDriveTrainSubsystem()::zeroRobotPosition),
          new DriveTrajectoryCommand(
            this.subsystemCollection.getDriveTrainSubsystem(),
            testTrajectories.traverseSimpleLeft)),
        new ButtonPressCommand(
          "driverController.y()",
          "testTrajectories.traverseSimpleLeft")).withTimeout(10.0)
    );
    // traverse turn 270 trajectory
    this.driverController.leftBumper().onTrue(
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new InstantCommand(subsystemCollection.getDriveTrainSubsystem()::zeroRobotPosition),
          new DriveTrajectoryCommand(
            this.subsystemCollection.getDriveTrainSubsystem(),
            testTrajectories.traverseTurn270)),
        new ButtonPressCommand(
          "driverController.leftBumper()",
          "testTrajectories.traverseTurn270")).withTimeout(10.0)
    );

    this.driverController.rightBumper().onTrue(
      new ParallelCommandGroup(
        new SequentialCommandGroup(
          new InstantCommand(subsystemCollection.getDriveTrainSubsystem()::zeroRobotPosition),
          new DriveTrajectoryCommand(
            this.subsystemCollection.getDriveTrainSubsystem(),
            testTrajectories.turn90)),
        new ButtonPressCommand(
          "driverController.rightBumper()",
          "testTrajectories.turn90")).withTimeout(10.0)
    );
  }

  /**
   * displays target game piece on smartdashboard
   */
  private void displayTargetGamePiece() {
    SmartDashboard.putBoolean("ConeMode", this.coDriverControllerGamePieceTarget == ChargedUpGamePiece.Cone);
    SmartDashboard.putBoolean("CubeMode", this.coDriverControllerGamePieceTarget == ChargedUpGamePiece.Cube);
    }
  
  /**
   * A method to do the transformation of current robot position to another position
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
  private void bindCommandsToCoDriverXboxButtons()
  {
    if(InstalledHardware.coDriverXboxControllerInstalled)
    {
      if(subsystemCollection.getArmSubsystem() != null) {
        // right bumper stow preset for arm
        this.coDriverController.rightBumper().onTrue(
          new ParallelCommandGroup(
            new ArmToLocationCommand(
              this.subsystemCollection.getArmSubsystem(),
              ArmLocation.ARM_STOW,
              this),
            new ButtonPressCommand(
              "coDriverController.rightBumper()",
              "arm stow")
            ).andThen(new RumbleCommand(coDriverControllerForRumbleOnly, Constants.rumbleTimeSeconds))
          );
        // left bumper grab preset for arm
        this.coDriverController.leftBumper().onTrue(
          new ParallelCommandGroup(
            new ArmToLocationCommand(
              this.subsystemCollection.getArmSubsystem(),
              ArmLocation.ARM_GRAB,
              this),
            new ButtonPressCommand(
              "coDriverController.leftBumper()",
              "arm grab")
            ).andThen(new RumbleCommand(coDriverControllerForRumbleOnly, Constants.rumbleTimeSeconds))
          );
        // y button press will move the arms to high score position
        this.coDriverController.y().onTrue(
          new ParallelCommandGroup(
            new ArmToLocationCommand(
              this.subsystemCollection.getArmSubsystem(),
              ArmLocation.ARM_HIGH_SCORE,
              this),
            new ButtonPressCommand(
              "coDriverController.y()",
              "arm score high")
            ).andThen(new RumbleCommand(coDriverControllerForRumbleOnly, Constants.rumbleTimeSeconds))
          );
          // b button press will move the arms to medium score position
        this.coDriverController.b().onTrue(
          new ParallelCommandGroup(
            new ArmToLocationCommand(
              this.subsystemCollection.getArmSubsystem(),
              ArmLocation.ARM_MED_SCORE,
              this),
            new ButtonPressCommand(
              "coDriverController.b()",
              "arm score medium")
            ).andThen(new RumbleCommand(coDriverControllerForRumbleOnly, Constants.rumbleTimeSeconds))
          );
        // a button press will drive the arms to low score position
        this.coDriverController.a().onTrue(
          new ParallelCommandGroup(
            new ArmToLocationCommand(
              this.subsystemCollection.getArmSubsystem(),
              ArmLocation.ARM_LOW_SCORE,
              this),
            new ButtonPressCommand(
              "coDriverController.a()",
              "arm score low")
            ).andThen(new RumbleCommand(coDriverControllerForRumbleOnly, Constants.rumbleTimeSeconds))
          );
      }
      // x button press will stop all
      this.coDriverController.x().onTrue(
        new ParallelCommandGroup(
          new AllStopCommand(
            this.subsystemCollection),
          new ButtonPressCommand(
            "coDriverController.x()",
            "!!!!!!!!!!!!!!!!!!!! ALL STOP !!!!!!!!!!!!!!!!!!!!!")
          )
        );

        if(subsystemCollection.getPickerSubsystem() != null) {
          // left trigger press will close the picker  
          this.coDriverController.leftTrigger().onTrue(
            new ParallelCommandGroup(
              new ManipulatePickerCommand(
                subsystemCollection.getPickerSubsystem(), false),
              new ButtonPressCommand(
              "coDriverController.leftTrigger()",
              "close the picker")
            )
          );
          // right trigger press will open the picker  
          this.coDriverController.rightTrigger().onTrue(
            new ParallelCommandGroup(
              new ManipulatePickerCommand(
                subsystemCollection.getPickerSubsystem(), true),
              new ButtonPressCommand(
              "coDriverController.rightTrigger()",
              "open the picker")
            )
          );
        }

        // Back button sets cube
        this.coDriverController.back().onTrue(
          new ParallelCommandGroup(
            new InstantCommand(subsystemCollection.getManualInputInterfaces()::setTargetGamePieceAsCube),
            new ButtonPressCommand(
              "coDriverController.back()",
              "set target as cube")
            )
          );

        // start button does sets cone
        this.coDriverController.start().onTrue(
          new ParallelCommandGroup(
            new InstantCommand(subsystemCollection.getManualInputInterfaces()::setTargetGamePieceAsCone),
            new ButtonPressCommand(
              "coDriverController.start()",
              "set target as cone")
            )
          );            

        // NOTE: subsystemCollection.getEveryBotPickerSubsystem()
        // THESE ARE TAKEN CARE OF IN DEFAULT COMMANDS
        // left trigger variable press will intake on the every bot picker 
        // right trigger variable press will expell on the every bot picker 
    }
  }
}
