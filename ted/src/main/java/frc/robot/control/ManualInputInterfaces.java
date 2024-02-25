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

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.common.FeederMode;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AllStopCommand;
import frc.robot.commands.ButtonPressCommand;
import frc.robot.commands.FeedNoteCommand;
import frc.robot.commands.IntakeNoteCommand;
import frc.robot.commands.RemoveNoteCommand;
import frc.robot.commands.ShooterShootCommand;
import frc.robot.commands.ShooterSpinUpCommand;

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

  /**
   * Get the next angle
   * @return incrementing or decrementing angle depending on positive or negative
   */
  public double getInputShooterAngle() 
  {
    System.out.println("<<<getInputShooterAngle>>>");
    // remember that the Y on xbox will be negative upward
    double stickInput = coDriverController.getRightY();
    double startAngle = this.subsystemCollection.getShooterSubsystem().getAngleDegrees();
    double updatedAngle = startAngle;
    if(stickInput > Constants.shooterControllerInputPositiveStickAngleIncrement){
      updatedAngle -= 2.0;
      System.out.println("decrementing angle from " + startAngle + " to " + updatedAngle);
    }
    else if (stickInput < Constants.shooterControllerInputNegativeStickAngleIncrement) {
      updatedAngle += 2.0;
      System.out.println("incrementing angle from " + startAngle + " to " + updatedAngle);
    }
    else {
      System.out.println("!!!NO shooter delta!!!");
    }
    return updatedAngle;
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

      if(this.subsystemCollection.isDriveTrainSubsystemAvailable()){
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

      if(this.subsystemCollection.isIntakeSubsystemAvailable() && 
       this.subsystemCollection.isFeederSubsystemAvailable()) {
        // b button will intake a note
        this.driverController.b().onTrue(
            new ParallelCommandGroup(
              new IntakeNoteCommand(this.subsystemCollection.getIntakeSubsystem()), 
              new FeedNoteCommand(this.subsystemCollection.getFeederSubsystem(), FeederMode.FeedToShooter), 
              new ButtonPressCommand(
                "driverController.b()",
                "intake")
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

      if(this.subsystemCollection.isShooterSubsystemAvailable() &&
         this.subsystemCollection.isFeederSubsystemAvailable()) {
        System.out.println("STARTING Registering this.driverController.a().whileTrue() ... ");
        this.driverController.a().whileTrue(
            new ParallelCommandGroup(
              new ShooterShootCommand(
                subsystemCollection.getShooterSubsystem(), 
                subsystemCollection.getFeederSubsystem()),
              new ButtonPressCommand(
                "driverController.a()",
                "Shoot at speed!!")
              )
          );
        System.out.println("FINISHED registering this.driverController.a().whileTrue() ... ");
      }

      if(this.subsystemCollection.isDriveTrainPowerSubsystemAvailable() && 
         this.subsystemCollection.isDriveTrainSubsystemAvailable()){
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

        // left trigger press will align robot on a target   
        this.driverController.leftTrigger().onTrue(
          new ParallelCommandGroup(
            //TODO create and add target driving mode here
            new InstantCommand(
              /*() -> subsystemCollection.getDriveTrainSubsystem().setSwerveDriveMode(SwerveDriveMode.TARGET_DRIVING)*/
            ),
            new ButtonPressCommand(
            "driverController.leftTrigger()",
            "align on target")
          )
        );

        // left trigger de-press will put drivetrain in normal drive mode  
        this.driverController.leftTrigger().onFalse(
          new ParallelCommandGroup(
            //
            new InstantCommand(
              /*() -> subsystemCollection.getDriveTrainSubsystem().setSwerveDriveMode(SwerveDriveMode.NORMAL_DRIVING)*/
            ),
            new ButtonPressCommand(
            "driverController.leftTrigger()",
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
          //TODO replace InstantCommand with actual driveFinePlacement command once it exist. 
            new InstantCommand()
          /**
          new DriveFinePlacementCommand(
            localDrive, 
            -1 * Constants.FinePlacementRotationalVelocity
            )*/
          ); 
        
        this.driverController.povLeft().whileTrue(
        //TODO replace InstantCommand with actual driveFinePlacement command once it exist. 
            new InstantCommand()
          /**  
          new DriveFinePlacementCommand(
            localDrive, 
            Constants.FinePlacementRotationalVelocity
            )
             */
          ); 
      }      
    }
  }
  
  /**
   * Will attach commands to the Co Driver XBox buttons 
   */
  private void bindCommandsToCoDriverXboxButtons()
  {
   //FeedNoteCommand feederToShooter = new FeedNoteCommand(subsystemCollection.getFeederSubsystem(), FeederMode.FeedToShooter);
   //FeedNoteCommand feederToDunker = new FeedNoteCommand(subsystemCollection.getFeederSubsystem(), FeederMode.FeedToDunker);
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

      if(this.subsystemCollection.isShooterSubsystemAvailable()) {
        this.coDriverController.y().onTrue(
          new SequentialCommandGroup(
            //TODO include an actual ShooterOuttake command here
            new ShooterSpinUpCommand(this.subsystemCollection.getShooterSubsystem()),
            new ShooterShootCommand(45, this.subsystemCollection.getShooterSubsystem(), this.subsystemCollection.getFeederSubsystem()),
            new ButtonPressCommand(
              "coDriverController.y()",
                "shoots the shooter")
            )
        );
      }

      if(this.subsystemCollection.isIntakeSubsystemAvailable()) {
        // remove the note
        this.coDriverController.b().onTrue(
          new ParallelCommandGroup(
            new RemoveNoteCommand(this.subsystemCollection.getIntakeSubsystem()),
            new ButtonPressCommand(
              "coDriverController.b()",
                "remove note")
            )
        );
      }

      this.coDriverController.a().onTrue(
        new ParallelCommandGroup(
          //TODO include an actual Climb command here
          new ButtonPressCommand(
            "coDriverController.a()",
              "[TEMPORARY FAKE] climb")
          )
      );

      this.coDriverController.rightBumper().onTrue(
          new ParallelCommandGroup(
            //TODO include an actual ShooterToLocation command here
            new ButtonPressCommand(
              "coDriverController.rightBumper()",
              "[TEMPORARY FAKE] stow")
            )
          );

      if(this.subsystemCollection.isFeederSubsystemAvailable()) {
          this.coDriverController.back().onTrue(
              new ParallelCommandGroup(
                new FeedNoteCommand(
                  this.subsystemCollection.getFeederSubsystem(),
                  FeederMode.FeedToShooter),
                new ButtonPressCommand(
                  "coDriverController.back()",
                  "send note to shooter"))
            );

          this.coDriverController.start().onTrue(
            new ParallelCommandGroup(
                new FeedNoteCommand(
                  this.subsystemCollection.getFeederSubsystem(),
                  FeederMode.FeedToDunker),
                new ButtonPressCommand(
                "coDriverController.start()",
                "send note to dunker"))
          );
      }
    }
  }
}
