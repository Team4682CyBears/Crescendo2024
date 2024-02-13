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
import frc.robot.commands.ShootAtSpeedCommand;
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
   * A method to get the arcade arm Z componet being input from humans
   * @return - a double value associated with the magnitude of the Z componet
   */
  public double getInputClimberArmsZ()
  {
    // use the co drivers right Z to represent the vertical movement
    // and multiply by -1.0 as xbox reports values flipped
    return -1.0 * coDriverController.getLeftY();
  }

  public double getInputShooterAngle() 
  {
    // use the co drivers right Z to represent the vertical movement
    // and multiply by -1.0 as xbox reports values flipped
    return coDriverController.getRightY();
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

      this.driverController.b().onTrue(
          new ParallelCommandGroup(
            new IntakeCommand(this.subsystemCollection),
            new ButtonPressCommand(
              "driverController.b()",
              "intake")
            )
          );

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
              new ShootAtSpeedCommand(subsystemCollection.getShooterSubsystem()),
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
      if(localDrive != null){
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
        this.driverController.leftTrigger().onTrue(
          new ParallelCommandGroup(
            new InstantCommand(
              () -> subsystemCollection.getDriveTrainSubsystem().setSwerveDriveMode(SwerveDriveMode.IMMOVABLE_STANCE)
            ),
            new ButtonPressCommand(
            "driverController.leftTrigger()",
            "immoveable stance")
          )
        );
        // right trigger de-press will put drivetrain in normal drive mode  
        this.driverController.leftTrigger().onFalse(
          new ParallelCommandGroup(
            new InstantCommand(
              () -> subsystemCollection.getDriveTrainSubsystem().setSwerveDriveMode(SwerveDriveMode.NORMAL_DRIVING)
            ),
            new ButtonPressCommand(
            "driverController.Trigger()",
            "normal driving")
          )
        );
        // right trigger press will ramp down drivetrain to reduced speed mode 
        this.driverController.rightTrigger().onTrue(
          new ParallelCommandGroup(
            new InstantCommand(subsystemCollection.getDriveTrainPowerSubsystem()::setReducedPowerReductionFactor,
            subsystemCollection.getDriveTrainPowerSubsystem()),
            new ButtonPressCommand(
            "driverController.rightTrigger()",
            "ramp down to reduced speed")
          )
        );
        // right trigger de-press will ramp up drivetrain to max speed
        this.driverController.rightTrigger().onFalse(
          new ParallelCommandGroup(
            new InstantCommand(subsystemCollection.getDriveTrainPowerSubsystem()::resetPowerReductionFactor,
            subsystemCollection.getDriveTrainPowerSubsystem()),
            new ButtonPressCommand(
            "driverController.rightTrigger()",
            "ramp up to default speed")
          )
        );
        // Dpad will control fine placement mode
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
    }
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

      this.coDriverController.y().onTrue(
        new ParallelCommandGroup(
          new ShooterScoreCommand(
            this.subsystemCollection),
          new ButtonPressCommand(
            "coDriverController.y()",
              "shooter outtake")
          )
      );

      this.coDriverController.b().onTrue(
        new ParallelCommandGroup(
          new AmpAndDunkerScoreCommand(
            this.subsystemCollection),
          new ButtonPressCommand(
            "coDriverController.b()",
              "score to amp or dunker")
          )
      );

      this.coDriverController.a().onTrue(
        new ParallelCommandGroup(
          new ClimbCommand(
            this.subsystemCollection),
          new ButtonPressCommand(
            "coDriverController.a()",
              "climb")
          )
      );

      this.coDriverController.rightBumper().onTrue(
          new ParallelCommandGroup(
            new ShooterToLocationCommand(
              this.subsystemCollection),
            new ButtonPressCommand(
              "coDriverController.rightBumper()",
              "stow")
            )
          );

          this.coDriverController.back().onTrue(
            new ParallelCommandGroup(
              new InstantCommand(subsystemCollection.getFeederSubsystem(), FeederMode.FeedToShooter),
              new ButtonPressCommand(
                "coDriverController.back()",
                "send note to shooter")
              )
            );

          this.coDriverController.start().onTrue(
          new ParallelCommandGroup(
            new InstantCommand(subsystemCollection.getFeederSubsystem(), FeederMode.FeedToDunker),
            new ButtonPressCommand(
              "coDriverController.start()",
              "send note to dunker")
            )
          );            

    }
  }
}
