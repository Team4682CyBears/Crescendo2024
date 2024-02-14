// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
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
import frc.robot.commands.RumbleCommand;
import frc.robot.common.TestTrajectories;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AllStopCommand;
import frc.robot.commands.ButtonPressCommand;
import frc.robot.commands.DriveToPointCommand;
import frc.robot.commands.DriveTrajectoryCommand;
import frc.robot.commands.ShooterShootCommand;
import frc.robot.commands.SteerMotorToAngleCommand;

public class ManualInputInterfaces {

  // sets joystick variables to joysticks
  private CommandXboxController driverController = new CommandXboxController(Constants.portDriverController); 
  private CommandXboxController coDriverController = new CommandXboxController(Constants.portCoDriverController);
  private XboxController coDriverControllerForRumbleOnly = new XboxController(Constants.portCoDriverController);

  // subsystems needed for inputs
  private SubsystemCollection subsystemCollection = null;

  /**
   * The constructor to build this 'manual input' conduit
   */
  public ManualInputInterfaces(SubsystemCollection currentCollection){
    subsystemCollection = currentCollection;
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

      if(subsystemCollection.getShooterSubsystem() != null) {
        System.out.println("STARTING Registering this.driverController.a().whileTrue() ... ");
        this.driverController.a().whileTrue(
            new ParallelCommandGroup(
              new ShooterShootCommand(subsystemCollection.getShooterSubsystem(), subsystemCollection.getFeederSubsystem()),
              new ButtonPressCommand(
                "driverController.a()",
                "Shoot at speed!!")
              )
          );
        System.out.println("FINISHED registering this.driverController.a().whileTrue() ... ");
      }

      if(subsystemCollection.getSteerMotorSubsystem() != null) {
        System.out.println("STARTING Registering this.driverController.y().whileTrue() ... ");
        this.driverController.y().whileTrue(
            new ParallelCommandGroup(
              new SteerMotorToAngleCommand(subsystemCollection.getSteerMotorSubsystem(), 0.0),
              new ButtonPressCommand(
                "driverController.y()",
                "SteerMotorToAngleCommand - 0.0 degrees")
              )
          );
        System.out.println("FINISHED registering this.driverController.y().whileTrue() ... ");

        System.out.println("STARTING Registering this.driverController.b().whileTrue() ... ");
        this.driverController.b().whileTrue(
            new ParallelCommandGroup(
              new SteerMotorToAngleCommand(subsystemCollection.getSteerMotorSubsystem(), 180.0),
              new ButtonPressCommand(
                "driverController.y()",
                "SteerMotorToAngleCommand - 180.0 degrees")
              )
          );
        System.out.println("FINISHED registering this.driverController.b().whileTrue() ... ");
      }


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

      // TODO - what control do we need??
    }
  }
}
