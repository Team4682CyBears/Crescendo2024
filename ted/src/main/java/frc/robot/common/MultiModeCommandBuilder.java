package frc.robot.common;

import java.util.HashMap;
import java.util.Map;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.ButtonPressCommand;
import frc.robot.commands.ClimberArmToPosition;
import frc.robot.commands.FeedNoteCommand;
import frc.robot.commands.ShooterIdleCommand;
import frc.robot.commands.ShooterSetAngleCommand;
import frc.robot.commands.ShooterShootCommand;
import frc.robot.control.SubsystemCollection;
import frc.robot.subsystems.ShooterAngleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public class MultiModeCommandBuilder {
    
    private SubsystemCollection subsystemCollection = null;
    private Command coDriverControllerButtonYSelectCommand = null;
    private Command coDriverControllerButtonBSelectCommand = null;
    private Command coDriverControllerButtonASelectCommand = null;
    private Command coDriverControllerPovDownSelectCommand = null;
    private Command coDriverControllerPovUpSelectCommand = null;

    /**
     * Construct a builder to create the various flavors of select commands that can be used
     * @param collection - the current subsystem collection
     */
    public MultiModeCommandBuilder(SubsystemCollection collection) {
        subsystemCollection = collection;
        this.coDriverControllerButtonYSelectCommand = this.buildCoDriverControllerLibraryCommand(MuitiModeOverloadedButtons.ButtonY);
        this.coDriverControllerButtonBSelectCommand = this.buildCoDriverControllerLibraryCommand(MuitiModeOverloadedButtons.ButtonB);
        this.coDriverControllerButtonASelectCommand = this.buildCoDriverControllerLibraryCommand(MuitiModeOverloadedButtons.ButtonA);
        this.coDriverControllerPovDownSelectCommand = this.buildCoDriverControllerLibraryCommand(MuitiModeOverloadedButtons.PovDown);
        this.coDriverControllerPovUpSelectCommand = this.buildCoDriverControllerLibraryCommand(MuitiModeOverloadedButtons.PovUp);
    }

    /**
     * A method to obtain the constructed select command for the co-driver controller overloaded button presses - ButtonY
     */
    public Command getCoDriverSelectCommandButtonY() {
        return coDriverControllerButtonYSelectCommand;
    } 

    /**
     * A method to obtain the constructed select command for the co-driver controller overloaded button presses - ButtonB
     */
    public Command getCoDriverSelectCommandButtonB() {
        return coDriverControllerButtonBSelectCommand;
    } 

    /**
     * A method to obtain the constructed select command for the co-driver controller overloaded button presses - ButtonA
     */
    public Command getCoDriverSelectCommandButtonA() {
        return coDriverControllerButtonASelectCommand;
    } 

    /**
     * A method to obtain the constructed select command for the co-driver controller overloaded button presses - PovDown
     */
    public Command getCoDriverSelectCommandPovDown() {
        return coDriverControllerPovDownSelectCommand;
    } 

    /**
     * A method to obtain the constructed select command for the co-driver controller overloaded button presses - PovUp
     */
    public Command getCoDriverSelectCommandPovUp() {
        return coDriverControllerPovUpSelectCommand;
    } 

    /***************************************************************************
     * PRIVATE
     ***************************************************************************/
    private Command buildCoDriverControllerLibraryCommand(MuitiModeOverloadedButtons targetButton) {

        Map<MultiModeCommandSelection, Command> multiModeCommandMap = new HashMap<MultiModeCommandSelection, Command>();

        // shove in the missing subsystem command ... so the selector can submit something when the subsystems are gonzo
        multiModeCommandMap.put(
            MultiModeCommandSelection.MissingSubsystem, 
            new ParallelCommandGroup(
                new ButtonPressCommand(
                "coDriverController.<any>()",
                "MissingSubsystemCommand")));

        // build up the various shooter library commands that can be created
        if(this.subsystemCollection.isShooterOutfeedSubsystemAvailable() &&
           this.subsystemCollection.isShooterAngleSubsystemAvailable() &&
           this.subsystemCollection.isFeederSubsystemAvailable()) {

            if(targetButton == MuitiModeOverloadedButtons.ButtonY) {
                // amp shoot high
                ParallelCommandGroup shotAmpHigh = new ParallelCommandGroup();
                shotAmpHigh.addCommands(
                    new ShooterShootCommand(
                    ShooterAngleSubsystem.positionToDegrees(ShooterPosition.AmpHigh),
                    ShooterIdleCommand.getOutfeedSpeedEnumInTargetMotorRpm(ShooterOutfeedSpeed.AmpHigh),
                    ShooterIdleCommand.getOutfeedSpeedEnumInTargetMotorRpm(ShooterOutfeedSpeed.AmpHigh),
                    this.subsystemCollection.getShooterOutfeedSubsystem(),
                    this.subsystemCollection.getShooterAngleSubsystem(),
                    this.subsystemCollection.getFeederSubsystem()),
                    new ButtonPressCommand(
                    "coDriverController.y()",
                        "AMP Shot High"));
                multiModeCommandMap.put(MultiModeCommandSelection.AmpScoreHigh, shotAmpHigh);
            }

            if(targetButton == MuitiModeOverloadedButtons.ButtonB) {
                // amp shoot medium
                ParallelCommandGroup shotAmpMedium = new ParallelCommandGroup();
                shotAmpMedium.addCommands(
                    new ShooterShootCommand(
                    ShooterAngleSubsystem.positionToDegrees(ShooterPosition.AmpMedium),
                    ShooterIdleCommand.getOutfeedSpeedEnumInTargetMotorRpm(ShooterOutfeedSpeed.AmpMedium),
                    ShooterIdleCommand.getOutfeedSpeedEnumInTargetMotorRpm(ShooterOutfeedSpeed.AmpMedium),
                    this.subsystemCollection.getShooterOutfeedSubsystem(),
                    this.subsystemCollection.getShooterAngleSubsystem(),
                    this.subsystemCollection.getFeederSubsystem()),
                    new ButtonPressCommand(
                    "coDriverController.b()",
                        "AMP Shot Medium"));
                multiModeCommandMap.put(MultiModeCommandSelection.AmpScoreMedium, shotAmpMedium);
            }

            if(targetButton == MuitiModeOverloadedButtons.ButtonA) {
                // amp shoot low
                ParallelCommandGroup shotAmpLow = new ParallelCommandGroup();
                shotAmpLow.addCommands(
                    new ShooterShootCommand(
                    ShooterAngleSubsystem.positionToDegrees(ShooterPosition.AmpMedium),
                    ShooterIdleCommand.getOutfeedSpeedEnumInTargetMotorRpm(ShooterOutfeedSpeed.AmpLow),
                    ShooterIdleCommand.getOutfeedSpeedEnumInTargetMotorRpm(ShooterOutfeedSpeed.AmpLow),
                    this.subsystemCollection.getShooterOutfeedSubsystem(),
                    this.subsystemCollection.getShooterAngleSubsystem(),
                    this.subsystemCollection.getFeederSubsystem()),
                    new ButtonPressCommand(
                    "coDriverController.a()",
                        "AMP Shot Medium"));
                multiModeCommandMap.put(MultiModeCommandSelection.AmpScoreLow, shotAmpLow);
            }

            if(targetButton == MuitiModeOverloadedButtons.ButtonY) {
                // speaker shoot redline
                ParallelCommandGroup shotSpeakerRedline = new ParallelCommandGroup();
                shotSpeakerRedline.addCommands(
                    new ShooterShootCommand(
                    ShooterAngleSubsystem.positionToDegrees(ShooterPosition.SpeakerRedlineDistance),
                    ShooterIdleCommand.getOutfeedSpeedEnumInTargetMotorRpm(ShooterOutfeedSpeed.SpeakerRedlineDistance),
                    ShooterIdleCommand.getOutfeedSpeedEnumInTargetMotorRpm(ShooterOutfeedSpeed.SpeakerRedlineDistance),
                    this.subsystemCollection.getShooterOutfeedSubsystem(),
                    this.subsystemCollection.getShooterAngleSubsystem(),
                    this.subsystemCollection.getFeederSubsystem()),
                    new ButtonPressCommand(
                    "coDriverController.y()",
                        "SPEAKER Shot Redline"));
                multiModeCommandMap.put(MultiModeCommandSelection.SpeakerScoreRedline, shotSpeakerRedline);
            }

            if(targetButton == MuitiModeOverloadedButtons.ButtonB) {
                // speaker shoot podium
                ParallelCommandGroup shotSpeakerPodium = new ParallelCommandGroup();
                shotSpeakerPodium.addCommands(
                    new ShooterShootCommand(
                    ShooterAngleSubsystem.positionToDegrees(ShooterPosition.SpeakerPodiumDistance),
                    ShooterIdleCommand.getOutfeedSpeedEnumInTargetMotorRpm(ShooterOutfeedSpeed.SpeakerPodiumDistance),
                    ShooterIdleCommand.getOutfeedSpeedEnumInTargetMotorRpm(ShooterOutfeedSpeed.SpeakerPodiumDistance),
                    this.subsystemCollection.getShooterOutfeedSubsystem(),
                    this.subsystemCollection.getShooterAngleSubsystem(),
                    this.subsystemCollection.getFeederSubsystem()),
                    new ButtonPressCommand(
                    "coDriverController.b()",
                        "SPEAKER Shot Podium"));
                multiModeCommandMap.put(MultiModeCommandSelection.SpeakerScorePodium, shotSpeakerPodium);
            }

            if(targetButton == MuitiModeOverloadedButtons.ButtonA) {
                // speaker shoot close
                ParallelCommandGroup shotSpeakerClose = new ParallelCommandGroup();
                shotSpeakerClose.addCommands(
                    new ShooterShootCommand(
                    ShooterAngleSubsystem.positionToDegrees(ShooterPosition.SpeakerCloseDistance),
                    ShooterIdleCommand.getOutfeedSpeedEnumInTargetMotorRpm(ShooterOutfeedSpeed.SpeakerCloseDistance),
                    ShooterIdleCommand.getOutfeedSpeedEnumInTargetMotorRpm(ShooterOutfeedSpeed.SpeakerCloseDistance),
                    this.subsystemCollection.getShooterOutfeedSubsystem(),
                    this.subsystemCollection.getShooterAngleSubsystem(),
                    this.subsystemCollection.getFeederSubsystem()),
                    new ButtonPressCommand(
                    "coDriverController.a()",
                        "SPEAKER Shot Close"));
                multiModeCommandMap.put(MultiModeCommandSelection.SpeakerScoreClose, shotSpeakerClose);
            }
        }

        if(this.subsystemCollection.isShooterAngleSubsystemAvailable() &&
           this.subsystemCollection.isClimberSubsystemAvailable()) {

            if(targetButton == MuitiModeOverloadedButtons.ButtonY) {
                // climber extend both with shooter flip up
                ParallelCommandGroup extendClimbers = new ParallelCommandGroup();
                extendClimbers.addCommands(
                    new SequentialCommandGroup(
                        new ShooterSetAngleCommand(
                            ShooterAngleSubsystem.positionToDegrees(ShooterPosition.ClimbStow),
                            this.subsystemCollection.getShooterAngleSubsystem()),
                        new ClimberArmToPosition(
                            this.subsystemCollection.getClimberSubsystem(),
                            ClimberArm.BothClimbers,
                            ClimberArmTargetPosition.FullDeploy)),
                    new ButtonPressCommand(
                        "coDriverController.y()",
                        "CLIMBER Extend"));
                multiModeCommandMap.put(MultiModeCommandSelection.ClimberExtend, extendClimbers);
            }

            if(targetButton == MuitiModeOverloadedButtons.ButtonB) {
                // climber full auto
                ParallelCommandGroup fullAutoClimbers = new ParallelCommandGroup();
                fullAutoClimbers.addCommands(
                    new SequentialCommandGroup(
                        new ShooterSetAngleCommand(
                            ShooterAngleSubsystem.positionToDegrees(ShooterPosition.ClimbStow),
                            this.subsystemCollection.getShooterAngleSubsystem()),
                        new ClimberArmToPosition(
                            this.subsystemCollection.getClimberSubsystem(),
                            ClimberArm.BothClimbers,
                            ClimberArmTargetPosition.FullDeploy),
                        // TODO move the robot forward
                        new ClimberArmToPosition(
                            this.subsystemCollection.getClimberSubsystem(),
                            ClimberArm.BothClimbers,
                            ClimberArmTargetPosition.HangRobot)),
                    new ButtonPressCommand(
                        "coDriverController.b()",
                        "CLIMBER Full Auto"));
                multiModeCommandMap.put(MultiModeCommandSelection.ClimberFullAuto, fullAutoClimbers);
            }

            if(targetButton == MuitiModeOverloadedButtons.ButtonA) {
                // climber retract
                ParallelCommandGroup retractClimbers = new ParallelCommandGroup();
                retractClimbers.addCommands(
                    new SequentialCommandGroup(
                        // TODO - THIS COULD GO CRUNCH IF NOT FIRST PUT AT THIS ANGLE ... BUT IF ITS NOT AT THIS ANGLE IT WOULD GO CRUNCH ANYWAY?!?!
                        new ShooterSetAngleCommand(
                            ShooterAngleSubsystem.positionToDegrees(ShooterPosition.ClimbStow),
                            this.subsystemCollection.getShooterAngleSubsystem()),
                        new ClimberArmToPosition(
                            this.subsystemCollection.getClimberSubsystem(),
                            ClimberArm.BothClimbers,
                            ClimberArmTargetPosition.FullRetract)),
                    new ButtonPressCommand(
                        "coDriverController.a()",
                        "CLIMBER Retract"));
                multiModeCommandMap.put(MultiModeCommandSelection.ClimberRetract, retractClimbers);
            }
        }

        if(this.subsystemCollection.isShooterAngleSubsystemAvailable()) {
            if(targetButton == MuitiModeOverloadedButtons.PovDown) {
                // stow shooter
                ParallelCommandGroup stowShooter = new ParallelCommandGroup();
                stowShooter.addCommands(
                    new ShooterSetAngleCommand(
                    ShooterAngleSubsystem.positionToDegrees(ShooterPosition.Stow),
                    this.subsystemCollection.getShooterAngleSubsystem()),
                new ButtonPressCommand(
                    "coDriverController.povDown()",
                    "Stow Shooter"));
                multiModeCommandMap.put(MultiModeCommandSelection.ShooterStow, stowShooter);

                // vertical shooter
                ParallelCommandGroup verticalShooter = new ParallelCommandGroup();
                verticalShooter.addCommands(
                    new ShooterSetAngleCommand(
                    ShooterAngleSubsystem.positionToDegrees(ShooterPosition.ClimbStow),
                    this.subsystemCollection.getShooterAngleSubsystem()),
                new ButtonPressCommand(
                    "coDriverController.povDown()",
                    "Vertical Shooter"));
                multiModeCommandMap.put(MultiModeCommandSelection.ShooterVertical, verticalShooter);
            }
        }

        if(this.subsystemCollection.isFeederSubsystemAvailable()) {
            if(targetButton == MuitiModeOverloadedButtons.PovUp) {
                // emergency feed
                ParallelCommandGroup emergencyFeed = new ParallelCommandGroup();
                emergencyFeed.addCommands(
                    new FeedNoteCommand(
                    this.subsystemCollection.getFeederSubsystem(),
                    FeederMode.FeedToShooter),
                    new ButtonPressCommand(
                        "coDriverController.povUp()",
                        "Feeder Manual Feed"));
                multiModeCommandMap.put(MultiModeCommandSelection.EmergencyFeederFeed, emergencyFeed);
            }
        }

        // assemble the select command
        Command rtnVal = null;
        if(targetButton == MuitiModeOverloadedButtons.ButtonA) {
            rtnVal = new SelectCommand<>(multiModeCommandMap, this::findProperModeCommandButtonA);
        }
        else if(targetButton == MuitiModeOverloadedButtons.ButtonB) {
            rtnVal = new SelectCommand<>(multiModeCommandMap, this::findProperModeCommandButtonB);
        }
        else if(targetButton == MuitiModeOverloadedButtons.ButtonY) {
            rtnVal = new SelectCommand<>(multiModeCommandMap, this::findProperModeCommandButtonY);
        }
        else if(targetButton == MuitiModeOverloadedButtons.PovDown) {
            rtnVal = new SelectCommand<>(multiModeCommandMap, this::findProperModeCommandPovDown);
        }
        else if(targetButton == MuitiModeOverloadedButtons.PovUp) {
            rtnVal = new SelectCommand<>(multiModeCommandMap, this::findProperModeCommandPovUp);
        }
        // TODO - else puke?

        return rtnVal;
    }

    /**
     * A method that knows how to locate the proper command - ButtonA
     * @return appropriate MultiModeCommandSelection for the button at hand
     */
    private MultiModeCommandSelection findProperModeCommandButtonA() {
        MultiModeCommandSelection selection = MultiModeCommandSelection.MissingSubsystem;
        if(this.subsystemCollection.isManualInputInterfacesAvailable()) {
            CoDriverMode currentMode = this.subsystemCollection.getManualInputInterfaces().getCoDriverMode();
            if(currentMode == CoDriverMode.AmpScore) {
                if(this.subsystemCollection.isShooterOutfeedSubsystemAvailable() &&
                   this.subsystemCollection.isShooterAngleSubsystemAvailable() &&
                   this.subsystemCollection.isFeederSubsystemAvailable()) {
                    selection = MultiModeCommandSelection.AmpScoreLow;
                }
            }
            else if(currentMode == CoDriverMode.ClimbDunk) {
                if(this.subsystemCollection.isShooterAngleSubsystemAvailable() &&
                   this.subsystemCollection.isClimberSubsystemAvailable()) {
                    selection = MultiModeCommandSelection.ClimberRetract;
                }
            }
            else if(currentMode == CoDriverMode.SpeakerScore) {
                if(this.subsystemCollection.isShooterOutfeedSubsystemAvailable() &&
                   this.subsystemCollection.isShooterAngleSubsystemAvailable() &&
                   this.subsystemCollection.isFeederSubsystemAvailable()) {
                    selection = MultiModeCommandSelection.SpeakerScoreClose;
                }
            }
        }
        return selection;
    }

    /**
     * A method that knows how to locate the proper command - ButtonB
     * @return appropriate MultiModeCommandSelection for the button at hand
     */
    private MultiModeCommandSelection findProperModeCommandButtonB() {
        MultiModeCommandSelection selection = MultiModeCommandSelection.MissingSubsystem;
        if(this.subsystemCollection.isManualInputInterfacesAvailable()) {
            CoDriverMode currentMode = this.subsystemCollection.getManualInputInterfaces().getCoDriverMode();
            if(currentMode == CoDriverMode.AmpScore) {
                if(this.subsystemCollection.isShooterOutfeedSubsystemAvailable() &&
                   this.subsystemCollection.isShooterAngleSubsystemAvailable() &&
                   this.subsystemCollection.isFeederSubsystemAvailable()) {
                    selection = MultiModeCommandSelection.AmpScoreMedium;
                }
            }
            else if(currentMode == CoDriverMode.ClimbDunk) {
                if(this.subsystemCollection.isShooterAngleSubsystemAvailable() &&
                   this.subsystemCollection.isClimberSubsystemAvailable()) {
                    selection = MultiModeCommandSelection.ClimberFullAuto;
                }
            }
            else if(currentMode == CoDriverMode.SpeakerScore) {
                if(this.subsystemCollection.isShooterOutfeedSubsystemAvailable() &&
                   this.subsystemCollection.isShooterAngleSubsystemAvailable() &&
                   this.subsystemCollection.isFeederSubsystemAvailable()) {
                    selection = MultiModeCommandSelection.SpeakerScorePodium;
                }
            }
        }
        return selection;
    }

    /**
     * A method that knows how to locate the proper command - ButtonY
     * @return appropriate MultiModeCommandSelection for the button at hand
     */
    private MultiModeCommandSelection findProperModeCommandButtonY() {
        MultiModeCommandSelection selection = MultiModeCommandSelection.MissingSubsystem;
        if(this.subsystemCollection.isManualInputInterfacesAvailable()) {
            CoDriverMode currentMode = this.subsystemCollection.getManualInputInterfaces().getCoDriverMode();
            if(currentMode == CoDriverMode.AmpScore) {
                if(this.subsystemCollection.isShooterOutfeedSubsystemAvailable() &&
                   this.subsystemCollection.isShooterAngleSubsystemAvailable() &&
                   this.subsystemCollection.isFeederSubsystemAvailable()) {
                    selection = MultiModeCommandSelection.AmpScoreHigh;
                }
            }
            else if(currentMode == CoDriverMode.ClimbDunk) {
                if(this.subsystemCollection.isShooterAngleSubsystemAvailable() &&
                   this.subsystemCollection.isClimberSubsystemAvailable()) {
                    selection = MultiModeCommandSelection.ClimberExtend;
                }
            }
            else if(currentMode == CoDriverMode.SpeakerScore) {
                if(this.subsystemCollection.isShooterOutfeedSubsystemAvailable() &&
                   this.subsystemCollection.isShooterAngleSubsystemAvailable() &&
                   this.subsystemCollection.isFeederSubsystemAvailable()) {
                    selection = MultiModeCommandSelection.SpeakerScoreRedline;
                }
            }
        }
        return selection;
    }

    /**
     * A method that knows how to locate the proper command - PovDown
     * @return appropriate MultiModeCommandSelection for the button at hand
     */
    private MultiModeCommandSelection findProperModeCommandPovDown() {
        MultiModeCommandSelection selection = MultiModeCommandSelection.MissingSubsystem;
        if(this.subsystemCollection.isManualInputInterfacesAvailable()) {
            CoDriverMode currentMode = this.subsystemCollection.getManualInputInterfaces().getCoDriverMode();
            if(currentMode == CoDriverMode.AmpScore) {
                if(this.subsystemCollection.isShooterAngleSubsystemAvailable()) {
                    selection = MultiModeCommandSelection.ShooterStow;
                }
            }
            else if(currentMode == CoDriverMode.ClimbDunk) {
                if(this.subsystemCollection.isShooterAngleSubsystemAvailable()) {
                    selection = MultiModeCommandSelection.ShooterVertical;
                }
            }
            else if(currentMode == CoDriverMode.SpeakerScore) {
                if(this.subsystemCollection.isShooterAngleSubsystemAvailable()) {
                    selection = MultiModeCommandSelection.ShooterStow;
                }
            }
        }
        return selection;
    }

    
    /**
     * A method that knows how to locate the proper command - PovUp
     * @return appropriate MultiModeCommandSelection for the button at hand
     */
    private MultiModeCommandSelection findProperModeCommandPovUp() {
        MultiModeCommandSelection selection = MultiModeCommandSelection.MissingSubsystem;
        if(this.subsystemCollection.isManualInputInterfacesAvailable()) {
            CoDriverMode currentMode = this.subsystemCollection.getManualInputInterfaces().getCoDriverMode();
            if(currentMode == CoDriverMode.AmpScore) {
                if(this.subsystemCollection.isFeederSubsystemAvailable()) {
                    selection = MultiModeCommandSelection.EmergencyFeederFeed;
                }
            }
            else if(currentMode == CoDriverMode.ClimbDunk) {
                // nothing for now!!
            }
            else if(currentMode == CoDriverMode.SpeakerScore) {
                if(this.subsystemCollection.isFeederSubsystemAvailable()) {
                    selection = MultiModeCommandSelection.EmergencyFeederFeed;
                }
            }
        }
        return selection;
    }

}
