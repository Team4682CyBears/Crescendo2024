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
import frc.robot.common.ShooterOutfeedSpeedProvider;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.*;

public class ManualInputInterfaces {

  // sets joystick variables to joysticks
  private CommandXboxController driverController = new CommandXboxController(Constants.portDriverController); 
  private XboxController driverControllerForRumbleOnly = new XboxController(Constants.portDriverController);
  private CommandXboxController coDriverController = new CommandXboxController(Constants.portCoDriverController);
  private XboxController coDriverControllerForRumbleOnly = new XboxController(Constants.portCoDriverController);
  private ShooterOutfeedSpeedProvider outfeedSpeedProvider = null;

  // subsystems needed for inputs
  private SubsystemCollection subsystemCollection = null;

  /**
   * The constructor to build this 'manual input' conduit
   */
  public ManualInputInterfaces(SubsystemCollection currentCollection){
    subsystemCollection = currentCollection;
    if (this.subsystemCollection.isShooterAngleSubsystemAvailable()){
      outfeedSpeedProvider = ShooterOutfeedSpeedProvider.getInstance(
        this.subsystemCollection.getShooterAngleSubsystem());  
    }
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
   * A method to get the arcade climber Z componet being input from humans
   * @return - a double value associated with the magnitude of the Z componet
   */
  public double getInputLeftClimberArmZ()
  {
    // use the co drivers right Z to represent the vertical movement
    // and multiply by -1.0 as xbox reports values flipped
    return -1.0 * coDriverController.getLeftY();
  }

  /**
   * A method to get the arcade climber Z componet being input from humans
   * @return - a double value associated with the magnitude of the Z componet
   */
  public double getInputRightClimberArmZ()
  {
    // use the co drivers right Z to represent the vertical movement
    // and multiply by -1.0 as xbox reports values flipped
    return -1.0 * coDriverController.getRightY();
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
        // b button will intake a note with rumble on note detected
        this.driverController.b().onTrue(
            new ParallelCommandGroup(
              new IntakeAndFeedNoteCommand(
                this.subsystemCollection.getIntakeSubsystem(),
                this.subsystemCollection.getFeederSubsystem(),
                FeederMode.FeedToShooter,
                this.driverControllerForRumbleOnly), 
              new ButtonPressCommand(
                "driverController.b()",
                "intake"))
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
        System.out.println("STARTING Registering this.driverController.rightTrigger().onTrue() ... ");
        this.driverController.rightTrigger().onTrue(
            new ParallelCommandGroup(
              new FeederLaunchNote(
                subsystemCollection.getFeederSubsystem(), FeederMode.FeedToShooter),
              new ButtonPressCommand(
                "driverController.rightTrigger()",
                "Shoot at speed!!")
              )
          );
        System.out.println("FINISHED registering this.driverController.rightTrigger().onTrue() ... ");
      }

      if(this.subsystemCollection.isIntakeSubsystemAvailable()) {
          this.driverController.y().onTrue(
            new ParallelCommandGroup(
              new RemoveNoteCommand(
                this.subsystemCollection.getIntakeSubsystem()),
              new ButtonPressCommand(
                "driverController.y()",
                "Remove Note")
              )
          );
      }

      if(this.subsystemCollection.isDriveTrainPowerSubsystemAvailable() && 
         this.subsystemCollection.isDriveTrainSubsystemAvailable()){
        // left bumper press will put drivetrain in X stance
        this.driverController.leftBumper().onTrue(
          new ParallelCommandGroup(
            new InstantCommand(
              () -> subsystemCollection.getDriveTrainSubsystem().setSwerveDriveMode(SwerveDriveMode.IMMOVABLE_STANCE)
            ),
            new ButtonPressCommand(
            "driverController.leftBumper()",
            "x-stance / immovable")
          )
        );
        // left bumper release will put drivetrain in normal drive mode  
        this.driverController.leftBumper().onFalse(
          new ParallelCommandGroup(
            new InstantCommand(
              () -> subsystemCollection.getDriveTrainSubsystem().setSwerveDriveMode(SwerveDriveMode.NORMAL_DRIVING)
            ),
            new ButtonPressCommand(
            "driverController.leftBumper().onFalse()",
            "back to normal driving")
          )
        );

        // right bumper press will put drivetrain in X stance
        this.driverController.rightBumper().onTrue(
          new ParallelCommandGroup(
            new InstantCommand(
              () -> subsystemCollection.getDriveTrainSubsystem().setSwerveDriveMode(SwerveDriveMode.IMMOVABLE_STANCE)
            ),
            new ButtonPressCommand(
            "driverController.rightBumper()",
            "x-stance / immovable")
          )
        );
        // right bumper release will put drivetrain in normal drive mode  
        this.driverController.rightBumper().onFalse(
          new ParallelCommandGroup(
            new InstantCommand(
              () -> subsystemCollection.getDriveTrainSubsystem().setSwerveDriveMode(SwerveDriveMode.NORMAL_DRIVING)
            ),
            new ButtonPressCommand(
            "driverController.rightBumper().onFalse()",
            "back to normal driving")
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

        // Dpad will control fine placement mode
        this.driverController.povRight().whileTrue(
          new DriveFinePlacementCommand(
            this.subsystemCollection.getDriveTrainSubsystem(), 
            -1 * Constants.FinePlacementRotationalVelocity
            )
          ); 
        
        this.driverController.povLeft().whileTrue(
            new DriveFinePlacementCommand(
            this.subsystemCollection.getDriveTrainSubsystem(), 
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

      if(this.subsystemCollection.isShooterAngleSubsystemAvailable()) {

        this.coDriverController.y().onTrue(
          new ParallelCommandGroup(
            // shoot at the current angle
            new ShooterSetAngleCommand(
              Constants.shooterAngleShootFromSpeaker,
              this.subsystemCollection.getShooterAngleSubsystem()),
            new ButtonPressCommand(
              "coDriverController.y()",
              "subwoffer shot")
              ));

        this.coDriverController.b().onTrue(
          new ParallelCommandGroup(
            // shoot at the current angle
            new ShooterSetAngleCommand(
              Constants.shooterAngleShootFromStage,
              this.subsystemCollection.getShooterAngleSubsystem()),
            new ButtonPressCommand(
              "coDriverController.b()",
              "podium/note shot")
              ));

        this.coDriverController.a().onTrue(
          new ParallelCommandGroup(
            // shoot at the current angle
            new ShooterSetAngleCommand(
              Constants.shooterAngleShootFromAmp,
              this.subsystemCollection.getShooterAngleSubsystem()),
            new ButtonPressCommand(
              "coDriverController.a()",
              "amp shot")
              ));

        this.coDriverController.back().onTrue(
          new ParallelCommandGroup(
            // shoot at the current angle
            new ShooterSetAngleCommand(
              Constants.shooterAngleStowDegrees,
              this.subsystemCollection.getShooterAngleSubsystem()),
            new ButtonPressCommand(
              "coDriverController.back()",
              "stow")
              ));

        // Auto Ranging stuff
        this.coDriverController.leftTrigger().whileTrue(
          new ParallelCommandGroup(
            new ShooterSetAngleWithVisionContinuousCommand(this.subsystemCollection.getCameraSubsystem(), this.subsystemCollection.getShooterAngleSubsystem()),
            new ButtonPressCommand(
              "coDriverController.leftTrigger()",
              "auto ranging mode on")
              ));        

        // angle change commands 
        // upward
        this.coDriverController.povUp().whileTrue(
          new ParallelCommandGroup(
            new ShooterSetAngleUntilLimitCommand(
              true,
              this.subsystemCollection.getShooterAngleSubsystem()),
            new ButtonPressCommand(
              "coDriverController.povUp()",
              "increment angle of shooter")));

        // downward
        this.coDriverController.povDown().whileTrue(
          new ParallelCommandGroup(
            new ShooterSetAngleUntilLimitCommand(
              false,
              this.subsystemCollection.getShooterAngleSubsystem()),
            new ButtonPressCommand(
              "coDriverController.povDown()",
              "deccrement angle of shooter")));
      }

      if(this.subsystemCollection.isShooterOutfeedSubsystemAvailable() &&
         this.subsystemCollection.isShooterAngleSubsystemAvailable() &&
         this.subsystemCollection.isFeederSubsystemAvailable()) {
        this.coDriverController.rightTrigger().onTrue(
            new ParallelCommandGroup(
              new ShooterSpinUpForeverCommand(
                subsystemCollection.getShooterOutfeedSubsystem(), 
                subsystemCollection.getFeederSubsystem(),
                () -> this.outfeedSpeedProvider.getShotSpeedForCurrentAngle(),
                false),
              new ButtonPressCommand(
                "coDriverController.rightTriggger()",
                "shooter spin-up command")
              )
            );
        this.coDriverController.rightTrigger().onFalse(
            new ParallelCommandGroup(
              new ShooterSpinUpReleaseCommand(
                subsystemCollection.getShooterOutfeedSubsystem()),
              new ButtonPressCommand(
                "coDriverController.rightTrigger().onFalse",
                "stop spin-up command")
              )
            );
      }
    }
  }
}
