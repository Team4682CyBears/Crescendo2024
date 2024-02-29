// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: ManualInputInterfaces.java
// Intent: Forms a class that grants access to driver controlled inputs.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.control;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.common.ClimberArm;
import frc.robot.common.ClimberArmTargetPosition;
import frc.robot.common.CoDriverMode;
import frc.robot.common.FeederMode;
import frc.robot.common.ShooterOutfeedSpeed;
import frc.robot.common.ShooterPosition;
import frc.robot.subsystems.ShooterAngleSubsystem;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.AllStopCommand;
import frc.robot.commands.ButtonPressCommand;
import frc.robot.commands.ClimberArmToPosition;
import frc.robot.commands.FeedNoteCommand;
import frc.robot.commands.IntakeAndFeedNoteCommand;
import frc.robot.commands.IntakeNoteCommand;
import frc.robot.commands.RemoveNoteCommand;
import frc.robot.commands.ShooterSetAngleCommand;
import frc.robot.commands.ShooterSetAngleDefaultCommand;
import frc.robot.commands.ShooterShootCommand;
import frc.robot.commands.ShooterSpinUpCommand;

public class ManualInputInterfaces {

  // sets joystick variables to joysticks
  private CommandXboxController driverController = new CommandXboxController(Constants.portDriverController); 
  private CommandXboxController coDriverController = new CommandXboxController(Constants.portCoDriverController);
  private XboxController coDriverControllerForRumbleOnly = new XboxController(Constants.portCoDriverController);

  // subsystems needed for inputs
  private SubsystemCollection subsystemCollection = null;
  private CoDriverMode currentCoDriverMode = CoDriverMode.SpeakerScore;
  private FeederMode currentFeederMode = FeederMode.FeedToShooter;
  private ShooterOutfeedSpeed currentDefaultOutfeedSpeedSelected = ShooterOutfeedSpeed.Stopped;

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
   * Method to return current co driver mode
   * @return the currently selected co-driver mode
   */
  public CoDriverMode getCoDriverMode() {
    return this.currentCoDriverMode;
  }

  /**
   * Method to return current feeder mode
   * @return the currently selected feeder mode
   */
  public FeederMode getFeederMode() {
    return this.currentFeederMode;
  }

  /**
   * Method to return current shooter outfeed speed
   * @return the currently selected outfeed speed
   */
  public ShooterOutfeedSpeed getShooterOutfeedSpeed() {
    return this.currentDefaultOutfeedSpeedSelected;
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
   * A method to get the co driver y left
   * @return - a double value associated with the magnitude of the y componet
   */
  public double getInputCoDriverLeftY()
  {
    // and multiply by -1.0 as xbox reports values flipped
    return -1.0 * coDriverController.getLeftY();
  }

  /**
   * A method to get the co driver y right
   * @return - a double value associated with the magnitude of the y componet
   */
  public double getInputCoDriverRightY()
  {
    // and multiply by -1.0 as xbox reports values flipped
    return -1.0 * coDriverController.getRightY();
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

  /*************************************************************************************
   * PRIVATE METHODS
   *************************************************************************************/

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

      // first the climber dunk mode
      if(this.subsystemCollection.isClimberSubsystemAvailable() &&
         this.subsystemCollection.isShooterAngleSubsystemAvailable()) {
        this.coDriverController.rightTrigger().onTrue(
          new ParallelCommandGroup(
            new InstantCommand(
              () -> this.setCoDriverMode(CoDriverMode.ClimbDunk)
            ),
            new InstantCommand(
              () -> this.setFeederMode(FeederMode.FeedToDunker)
            ),
            new ButtonPressCommand(
            "coDriverController.rightTrigger()",
            "SET Climb / Dunk Mode!")
          )
        );
      }

      // second the speaker shoot mode
      if(this.subsystemCollection.isShooterAngleSubsystemAvailable() &&
         this.subsystemCollection.isShooterOutfeedSubsystemAvailable()) {
        this.coDriverController.rightBumper().onTrue(
          new ParallelCommandGroup(
            new InstantCommand(
              () -> this.setCoDriverMode(CoDriverMode.SpeakerScore)
            ),
            new InstantCommand(
              () -> this.setFeederMode(FeederMode.FeedToShooter)
            ),
            new ButtonPressCommand(
            "coDriverController.rightTrigger()",
            "SET Climb / Dunk Mode!")
          )
        );
      }

      // third the amp shoot mode
      if(this.subsystemCollection.isShooterAngleSubsystemAvailable() &&
         this.subsystemCollection.isShooterOutfeedSubsystemAvailable()) {
        this.coDriverController.rightBumper().onTrue(
          new ParallelCommandGroup(
            new InstantCommand(
              () -> this.setCoDriverMode(CoDriverMode.AmpScore)
            ),
            new InstantCommand(
              () -> this.setFeederMode(FeederMode.FeedToShooter)
            ),
            new ButtonPressCommand(
            "coDriverController.rightTrigger()",
            "SET Climb / Dunk Mode!")
          )
        );
      }

      // wire up the shooter idle / stopped default - only when the outfeed subsystem is available
      if(this.subsystemCollection.isShooterOutfeedSubsystemAvailable()) {       
        this.driverController.back().onTrue(
          new ParallelCommandGroup(
            new InstantCommand(
              () -> this.setDefaultOutfeedSpeedToIdle() // this method will only have effect when the proper modes are selected
            ),
            new ButtonPressCommand(
              "coDriverController.back()",
              "SET default outfeed speed to idle")
            )
          );
        this.driverController.start().onTrue(
          new ParallelCommandGroup(
            new InstantCommand(
              () -> this.setDefaultOutfeedSpeedToStopped() // this method will only have effect when the proper modes are selected
            ),
            new ButtonPressCommand(
              "coDriverController.back()",
              "set to stopped")
            )
          );
      }

      // do generic Y command - command to run determined at run time
      this.coDriverController.y().onTrue(
        new InstantCommand(() -> this.buildCoDriverYButtonCommands())
      );

      // do generic B command - command to run determined at run time
      this.coDriverController.b().onTrue(
        new InstantCommand(() -> this.buildCoDriverBButtonCommands())
      );

      // do generic A command - command to run determined at run time
      this.coDriverController.a().onTrue(
        new InstantCommand(() -> this.buildCoDriverAButtonCommands())
      );

      // do generic dpad down command - command to run determined at run time
      this.driverController.povDown().onTrue(
        new InstantCommand(() -> this.buildCoDriverPovDownButtonCommands())
      );

      // do generic dpad up command - command to run determined at run time
      this.driverController.povUp().onTrue(
        new InstantCommand(() -> this.buildCoDriverPovUpButtonCommands())
      );
    }
  }

  /**
   * Update the current mode selected
   * @param updatedMode - new mode to use
   */
  private void setCoDriverMode(CoDriverMode updatedMode) {
    this.currentCoDriverMode = updatedMode;
  }

  /**
   * Update the default outfeed speed to idle - only when proper modes are selected
   */
  private void setDefaultOutfeedSpeedToIdle() {
    if(this.currentCoDriverMode == CoDriverMode.SpeakerScore) {
      this.currentDefaultOutfeedSpeedSelected = ShooterOutfeedSpeed.SpeakerIdle;
    }
    else if(this.currentCoDriverMode == CoDriverMode.SpeakerScore) {
      this.currentDefaultOutfeedSpeedSelected = ShooterOutfeedSpeed.AmpIdle;
    }
  }

  /**
   * Update the default outfeed speed to stopped - only when proper modes are selected
   */
  private void setDefaultOutfeedSpeedToStopped() {
    if(this.currentCoDriverMode == CoDriverMode.SpeakerScore) {
      this.currentDefaultOutfeedSpeedSelected = ShooterOutfeedSpeed.Stopped;
    }
    else if(this.currentCoDriverMode == CoDriverMode.SpeakerScore) {
      this.currentDefaultOutfeedSpeedSelected = ShooterOutfeedSpeed.AmpIdle;
    }
  }

  /**
   * Update the current mode selected
   * @param updatedMode - new mode to use
   */
  private void setFeederMode(FeederMode updatedMode) {
    this.currentFeederMode = updatedMode;
  }

  /**
   * A method that understands how to build the proper co-driver Y button commands
   * based on the currently selected modes
   * @return A parallel command on what will be done
   */
  private ParallelCommandGroup buildCoDriverYButtonCommands() {
    CoDriverMode currentDriverMode = this.getCoDriverMode();
    ParallelCommandGroup group = new ParallelCommandGroup();
    if(currentDriverMode == CoDriverMode.AmpScore &&
       this.subsystemCollection.isShooterOutfeedSubsystemAvailable() &&
       this.subsystemCollection.isShooterAngleSubsystemAvailable() &&
       this.subsystemCollection.isFeederSubsystemAvailable()) {
      double desiredSpeed = ManualInputInterfaces.getOutfeedSpeedEnumInTargetMotorRpm(ShooterOutfeedSpeed.AmpHigh);
      double desiredAngle = ShooterAngleSubsystem.positionToDegrees(ShooterPosition.AmpHigh);
      group.addCommands(
        new ShooterShootCommand(
          desiredAngle,
          desiredSpeed,
          desiredSpeed,
          this.subsystemCollection.getShooterOutfeedSubsystem(),
          this.subsystemCollection.getShooterAngleSubsystem(),
          this.subsystemCollection.getFeederSubsystem()),
        new ButtonPressCommand(
          "coDriverController.y()",
            "AMP Shot High")
      );
    }
    else if(currentDriverMode == CoDriverMode.ClimbDunk &&
       this.subsystemCollection.isShooterAngleSubsystemAvailable() &&
       this.subsystemCollection.isClimberSubsystemAvailable()) {
      double desiredAngle = ShooterAngleSubsystem.positionToDegrees(ShooterPosition.ClimbStow);
      group.addCommands(
        new SequentialCommandGroup(
          new ShooterSetAngleCommand(
            desiredAngle,
            this.subsystemCollection.getShooterAngleSubsystem()),
          new ClimberArmToPosition(
            this.subsystemCollection.getClimberSubsystem(),
            ClimberArm.BothClimbers,
            ClimberArmTargetPosition.FullDeploy)
        ),
        new ButtonPressCommand(
          "coDriverController.y()",
            "CLIMBER Extend")
      );
    }
    else if(currentDriverMode == CoDriverMode.SpeakerScore &&
       this.subsystemCollection.isShooterOutfeedSubsystemAvailable() &&
       this.subsystemCollection.isShooterAngleSubsystemAvailable() &&
       this.subsystemCollection.isFeederSubsystemAvailable()) {
      double desiredSpeed = ManualInputInterfaces.getOutfeedSpeedEnumInTargetMotorRpm(ShooterOutfeedSpeed.SpeakerRedlineDistance);
      double desiredAngle = ShooterAngleSubsystem.positionToDegrees(ShooterPosition.SpeakerRedlineDistance);
      group.addCommands(
        new ShooterShootCommand(
          desiredAngle,
          desiredSpeed,
          desiredSpeed,
          this.subsystemCollection.getShooterOutfeedSubsystem(),
          this.subsystemCollection.getShooterAngleSubsystem(),
          this.subsystemCollection.getFeederSubsystem()),
        new ButtonPressCommand(
          "coDriverController.y()",
            "SPEAKER Shot Redline")
      );
    }
    else {
      group.addCommands(
        new ButtonPressCommand(
          "coDriverController.y()",
          "<------------- NO OP!!! -------------->")
      );
    }
    return group;
  }

  /**
   * A method that understands how to build the proper co-driver B button commands
   * based on the currently selected modes
   * @return A parallel command on what will be done
   */
  private ParallelCommandGroup buildCoDriverBButtonCommands() {
    CoDriverMode currentDriverMode = this.getCoDriverMode();
    ParallelCommandGroup group = new ParallelCommandGroup();
    if(currentDriverMode == CoDriverMode.AmpScore &&
       this.subsystemCollection.isShooterOutfeedSubsystemAvailable() &&
       this.subsystemCollection.isShooterAngleSubsystemAvailable() &&
       this.subsystemCollection.isFeederSubsystemAvailable()) {
      double desiredSpeed = ManualInputInterfaces.getOutfeedSpeedEnumInTargetMotorRpm(ShooterOutfeedSpeed.AmpMedium);
      double desiredAngle = ShooterAngleSubsystem.positionToDegrees(ShooterPosition.AmpMedium);
      group.addCommands(
        new ShooterShootCommand(
          desiredAngle,
          desiredSpeed,
          desiredSpeed,
          this.subsystemCollection.getShooterOutfeedSubsystem(),
          this.subsystemCollection.getShooterAngleSubsystem(),
          this.subsystemCollection.getFeederSubsystem()),
        new ButtonPressCommand(
          "coDriverController.b()",
            "AMP Shot Medium")
      );
    }
    else if(currentDriverMode == CoDriverMode.ClimbDunk &&
       this.subsystemCollection.isShooterAngleSubsystemAvailable() &&
       this.subsystemCollection.isClimberSubsystemAvailable()) {
      double desiredAngle = ShooterAngleSubsystem.positionToDegrees(ShooterPosition.ClimbStow);
      group.addCommands(
        new SequentialCommandGroup(
          new ShooterSetAngleCommand(
            desiredAngle,
            this.subsystemCollection.getShooterAngleSubsystem()),
          new ClimberArmToPosition(
            this.subsystemCollection.getClimberSubsystem(),
            ClimberArm.BothClimbers,
            ClimberArmTargetPosition.FullDeploy),
          // TODO move the robot forward
          new ClimberArmToPosition(
            this.subsystemCollection.getClimberSubsystem(),
            ClimberArm.BothClimbers,
            ClimberArmTargetPosition.HangRobot)
        ),
        new ButtonPressCommand(
          "coDriverController.b()",
            "CLIMBER Full Auto")
      );
    }
    else if(currentDriverMode == CoDriverMode.SpeakerScore &&
       this.subsystemCollection.isShooterOutfeedSubsystemAvailable() &&
       this.subsystemCollection.isShooterAngleSubsystemAvailable() &&
       this.subsystemCollection.isFeederSubsystemAvailable()) {
      double desiredSpeed = ManualInputInterfaces.getOutfeedSpeedEnumInTargetMotorRpm(ShooterOutfeedSpeed.SpeakerPodiumDistance);
      double desiredAngle = ShooterAngleSubsystem.positionToDegrees(ShooterPosition.SpeakerPodiumDistance);
      group.addCommands(
        new ShooterShootCommand(
          desiredAngle,
          desiredSpeed,
          desiredSpeed,
          this.subsystemCollection.getShooterOutfeedSubsystem(),
          this.subsystemCollection.getShooterAngleSubsystem(),
          this.subsystemCollection.getFeederSubsystem()),
        new ButtonPressCommand(
          "coDriverController.b()",
            "SPEAKER Shot Podium")
      );
    }
    else {
      group.addCommands(
        new ButtonPressCommand(
          "coDriverController.b()",
          "<------------- NO OP!!! -------------->")
      );
    }
    return group;
  }
  
  /**
   * A method that understands how to build the proper co-driver A button commands
   * based on the currently selected modes
   * @return A parallel command on what will be done
   */
  private ParallelCommandGroup buildCoDriverAButtonCommands() {
    CoDriverMode currentDriverMode = this.getCoDriverMode();
    ParallelCommandGroup group = new ParallelCommandGroup();
    if(currentDriverMode == CoDriverMode.AmpScore &&
       this.subsystemCollection.isShooterOutfeedSubsystemAvailable() &&
       this.subsystemCollection.isShooterAngleSubsystemAvailable() &&
       this.subsystemCollection.isFeederSubsystemAvailable()) {
      double desiredSpeed = ManualInputInterfaces.getOutfeedSpeedEnumInTargetMotorRpm(ShooterOutfeedSpeed.AmpLow);
      double desiredAngle = ShooterAngleSubsystem.positionToDegrees(ShooterPosition.AmpLow);
      group.addCommands(
        new ShooterShootCommand(
          desiredAngle,
          desiredSpeed,
          desiredSpeed,
          this.subsystemCollection.getShooterOutfeedSubsystem(),
          this.subsystemCollection.getShooterAngleSubsystem(),
          this.subsystemCollection.getFeederSubsystem()),
        new ButtonPressCommand(
          "coDriverController.a()",
            "AMP Shot Low")
      );
    }
    else if(currentDriverMode == CoDriverMode.ClimbDunk &&
       this.subsystemCollection.isShooterAngleSubsystemAvailable() &&
       this.subsystemCollection.isClimberSubsystemAvailable()) {
      double desiredAngle = ShooterAngleSubsystem.positionToDegrees(ShooterPosition.ClimbStow);
      group.addCommands(
        new SequentialCommandGroup(
          // TODO - THIS COULD GO CRUNCH IF NOT FIRST PUT AT THIS ANGLE ... BUT IF ITS NOT AT THIS ANGLE IT WOULD GO CRUNCH ANYWAY!!!
          new ShooterSetAngleCommand(
            desiredAngle,
            this.subsystemCollection.getShooterAngleSubsystem()),
          new ClimberArmToPosition(
            this.subsystemCollection.getClimberSubsystem(),
            ClimberArm.BothClimbers,
            ClimberArmTargetPosition.HangRobot)
        ),
        new ButtonPressCommand(
          "coDriverController.a()",
            "CLIMBER Retract")
      );
    }
    else if(currentDriverMode == CoDriverMode.SpeakerScore &&
       this.subsystemCollection.isShooterOutfeedSubsystemAvailable() &&
       this.subsystemCollection.isShooterAngleSubsystemAvailable() &&
       this.subsystemCollection.isFeederSubsystemAvailable()) {
      double desiredSpeed = ManualInputInterfaces.getOutfeedSpeedEnumInTargetMotorRpm(ShooterOutfeedSpeed.SpeakerCloseDistance);
      double desiredAngle = ShooterAngleSubsystem.positionToDegrees(ShooterPosition.SpeakerCloseDistance);
      group.addCommands(
        new ShooterShootCommand(
          desiredAngle,
          desiredSpeed,
          desiredSpeed,
          this.subsystemCollection.getShooterOutfeedSubsystem(),
          this.subsystemCollection.getShooterAngleSubsystem(),
          this.subsystemCollection.getFeederSubsystem()),
        new ButtonPressCommand(
          "coDriverController.a()",
            "SPEAKER Shot Close")
      );
    }
    else {
      group.addCommands(
        new ButtonPressCommand(
          "coDriverController.a()",
          "<------------- NO OP!!! -------------->")
      );
    }
    return group;
  }

  /**
   * A method that understands how to build the proper co-driver POV Down commands
   * based on the currently selected modes
   * @return A parallel command on what will be done
   */
  private ParallelCommandGroup buildCoDriverPovDownButtonCommands() {
    CoDriverMode currentDriverMode = this.getCoDriverMode();
    ParallelCommandGroup group = new ParallelCommandGroup();
    if((currentDriverMode == CoDriverMode.AmpScore || currentDriverMode == CoDriverMode.SpeakerScore) &&
       this.subsystemCollection.isShooterAngleSubsystemAvailable()) {
      double desiredAngle = ShooterAngleSubsystem.positionToDegrees(ShooterPosition.Stow);
      group.addCommands(
        new ShooterSetAngleCommand(
          desiredAngle,
          this.subsystemCollection.getShooterAngleSubsystem()),
        new ButtonPressCommand(
          "coDriverController.povDown()",
            "Stow Shooter")
      );
    }
    else {
      group.addCommands(
        new ButtonPressCommand(
          "coDriverController.povDown()",
          "<------------- NO OP!!! -------------->")
      );
    }
    return group;
  }

  /**
   * A method that understands how to build the proper co-driver POV Up commands
   * based on the currently selected modes
   * @return A parallel command on what will be done
   */
  private ParallelCommandGroup buildCoDriverPovUpButtonCommands() {
    CoDriverMode currentDriverMode = this.getCoDriverMode();
    ParallelCommandGroup group = new ParallelCommandGroup();
    if((currentDriverMode == CoDriverMode.ClimbDunk) &&
       this.subsystemCollection.isShooterAngleSubsystemAvailable()) {
      double desiredAngle = ShooterAngleSubsystem.positionToDegrees(ShooterPosition.ClimbStow);
      group.addCommands(
        new ShooterSetAngleCommand(
          desiredAngle,
          this.subsystemCollection.getShooterAngleSubsystem()),
        new ButtonPressCommand(
          "coDriverController.povUp()",
            "Climb Shooter")
      );
    }
    else {
      group.addCommands(
        new ButtonPressCommand(
          "coDriverController.povUp()",
          "<------------- NO OP!!! -------------->")
      );
    }
    return group;
  }

  /**
   * A method to translate shooter positions into degrees
   * @param position
   * @return speed of target motors
   */
  private static double getOutfeedSpeedEnumInTargetMotorRpm(ShooterOutfeedSpeed outfeedSpeed){
    double speed = Constants.shooterSpeedRpmStopped;
    switch (outfeedSpeed) {
      case Stopped:
        speed = Constants.shooterSpeedRpmStopped;
        break;
      case SpeakerIdle:
        speed = Constants.shooterSpeedRpmSpeakerIdle;
        break;
      case SpeakerCloseDistance:
        speed = Constants.shooterSpeedRpmSpeakerCloseDistance;
        break;
      case SpeakerPodiumDistance:
        speed = Constants.shooterSpeedRpmSpeakerPodiumDistance;
        break;
      case SpeakerRedlineDistance:
        speed = Constants.shooterSpeedRpmSpeakerRedlineDistance;
        break;
      case AmpLow:
        speed = Constants.shooterSpeedRpmAmpLow;
        break;
      case AmpMedium:
        speed = Constants.shooterSpeedRpmAmpMedium;
        break;
      case AmpHigh:
        speed = Constants.shooterSpeedRpmAmpHigh;
        break;
      default:
        speed = Constants.shooterSpeedRpmStopped;
        break;
    }
    return speed;
  }


}
