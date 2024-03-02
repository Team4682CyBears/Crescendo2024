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
import frc.robot.commands.IntakeAndFeedNoteCommand;
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
  private double rightClimberSpeed = 0.0;

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
  public double getInputLeftClimberArmZ()
  {
    // use the co drivers right Z to represent the vertical movement
    // and multiply by -1.0 as xbox reports values flipped
    return -1.0 * coDriverController.getLeftY();
  }

  /**
   * A method to get the arcade arm Z componet being input from humans
   * @return - a double value associated with the magnitude of the Z componet
   */
  public double getRightClimberArmZ()
  {
    return this.rightClimberSpeed;
  }

  /**
   * Get the angle increment
   * @return incrementing or decrementing angle depending on positive or negative
   */
  public double getInputShooterAngleIncrement() 
  {
    // remember that the Y on xbox will be negative upward
    double stickInput = coDriverController.getRightY();
    double updatedAngle = 0.0; 
    if(stickInput > Constants.shooterControllerInputPositiveStickAngleIncrement){
      updatedAngle = -Constants.shooterAngleStickIncrementMagnitude;
    }
    else if (stickInput < Constants.shooterControllerInputNegativeStickAngleIncrement) {
      updatedAngle = Constants.shooterAngleStickIncrementMagnitude;
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
              new IntakeAndFeedNoteCommand(
                this.subsystemCollection.getIntakeSubsystem(),
                this.subsystemCollection.getFeederSubsystem(),
                FeederMode.FeedToShooter), 
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

      if(this.subsystemCollection.isShooterOutfeedSubsystemAvailable() &&
         this.subsystemCollection.isFeederSubsystemAvailable()) {
        System.out.println("STARTING Registering this.driverController.a().whileTrue() ... ");
        this.driverController.a().whileTrue(
            new ParallelCommandGroup(
              new ShooterShootCommand(
                subsystemCollection.getShooterOutfeedSubsystem(), 
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

        // right trigger press will align robot on a target   
        this.driverController.rightTrigger().onTrue(
          new ParallelCommandGroup(
            //TODO create and add target driving mode here
            new InstantCommand(
              /*() -> subsystemCollection.getDriveTrainSubsystem().setSwerveDriveMode(SwerveDriveMode.TARGET_DRIVING)*/
            ),
            new ButtonPressCommand(
            "driverController.rightTrigger()",
            "align on target")
          )
        );

        // right trigger de-press will put drivetrain in normal drive mode  
        this.driverController.rightTrigger().onFalse(
          new ParallelCommandGroup(
            //
            new InstantCommand(
              /*() -> subsystemCollection.getDriveTrainSubsystem().setSwerveDriveMode(SwerveDriveMode.NORMAL_DRIVING)*/
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

      if(this.subsystemCollection.isShooterOutfeedSubsystemAvailable() && this.subsystemCollection.isFeederSubsystemAvailable()) {
        this.coDriverController.y().onTrue(
          new ParallelCommandGroup(
            // shoot at the current angle
            new ShooterShootCommand(this.subsystemCollection.getShooterOutfeedSubsystem(), this.subsystemCollection.getFeederSubsystem()),
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

      this.coDriverController.povUp().onTrue(
          new InstantCommand(() -> this.setRightClimberSpeedPositive())
      );
      this.coDriverController.povUp().onFalse(
          new InstantCommand(() -> this.setRightClimberSpeedZero())
      );
      this.coDriverController.povDown().onTrue(
          new InstantCommand(() -> this.setRightClimberSpeedNegative())
      );
      this.coDriverController.povDown().onFalse(
          new InstantCommand(() -> this.setRightClimberSpeedZero())
      );
    }
  }

  private void setRightClimberSpeedPositive() {
    this.rightClimberSpeed = Constants.climberArmUpDefaultSpeed;
  }
  private void setRightClimberSpeedNegative() {
    this.rightClimberSpeed = Constants.climberArmDownDefaultSpeed;
  }
  private void setRightClimberSpeedZero() {
    this.rightClimberSpeed = 0.0;
  }
}
