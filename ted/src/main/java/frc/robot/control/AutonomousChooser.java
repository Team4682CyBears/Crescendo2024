// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: AutonomousChooser.java
// Intent: Allows auto mode routine to be selected from shuffleboard
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.control;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import frc.robot.commands.ShooterSpinUpAutoCommand;
import frc.robot.commands.ButtonPressCommand;
import frc.robot.commands.IntakeAndFeedNoteCommand;
import frc.robot.commands.RemoveNoteCommand;
import frc.robot.commands.ShooterSetAngleCommand;
import frc.robot.commands.ShooterSetAngleWithVisionOneShotCommand;
import frc.robot.commands.UseFusedVisionInAutoCommand;
import frc.robot.common.FeederMode;
import frc.robot.subsystems.CameraSubsystem;
import frc.robot.commands.FeederLaunchNote;

/**
* a class for choosing different auto modes from shuffleboard
*/
public class AutonomousChooser {
    private SubsystemCollection subsystems;
    private final SendableChooser<AutonomousPath> autonomousPathChooser = new SendableChooser<>();

    private Command blue123SourceSide;
    private Command red123SourceSide;
    private Command blueWing;
    private Command blueFourNote;
    private Command blueFiveNote;
    private Command twoNote;
    private Command redWing;
    private Command redFourNote;
    private Command redFiveNote;
    private Command doNothing;
    /**
     * Constructor for AutonomousChooser
     * @param subsystems - the SubsystemCollection
     */
    public AutonomousChooser(SubsystemCollection subsystems){
        this.subsystems = subsystems;

        if (subsystems.isDriveTrainPowerSubsystemAvailable() &&
        subsystems.isIntakeSubsystemAvailable() && subsystems.isFeederSubsystemAvailable() &&
        subsystems.isShooterAngleSubsystemAvailable() && subsystems.isShooterOutfeedSubsystemAvailable()){

        autonomousPathChooser.setDefaultOption("Two Note", AutonomousPath.TWONOTE);
        autonomousPathChooser.addOption("BLUE 123 Source Side", AutonomousPath.BLUE123);
        autonomousPathChooser.addOption("RED 123 Source Side", AutonomousPath.RED123);
        autonomousPathChooser.addOption("BLUE Wing", AutonomousPath.BLUEWING);
        autonomousPathChooser.addOption("BLUE Four Note", AutonomousPath.BLUEFOURNOTE);
        autonomousPathChooser.addOption("BLUE 5 note", AutonomousPath.BLUEFIVENOTE);
        autonomousPathChooser.addOption("RED Wing", AutonomousPath.REDWING);
        autonomousPathChooser.addOption("RED Four Note", AutonomousPath.REDFOURNOTE);
        autonomousPathChooser.addOption("RED 5 note", AutonomousPath.REDFIVENOTE);
        autonomousPathChooser.addOption("Do Nothing", AutonomousPath.DONOTHING);
        SmartDashboard.putData(autonomousPathChooser);

        this.blue123SourceSide = getBlue123SourceSide();
        this.red123SourceSide = getRed123SourceSide();
        this.blueWing = getBlueWing();
        this.blueFourNote = getBlueFourNote();
        this.blueFiveNote = getBlueFiveNote();
        this.twoNote = getTwoNote();
        this.redWing = getRedWing();
        this.redFourNote = getRedFourNote();
        this.redFiveNote = getRedFiveNote();
        this.doNothing = getDoNothing();

        }
        else{
            System.out.println(">>>>> NO auto routine becuase missing subsystems");
        }
    }

     /**
      * returns the path planner auto to be used in auto period
      * @return command
      */
    public Command getAutoPath() {
        switch (autonomousPathChooser.getSelected()) {
            case BLUE123 :
                return this.blue123SourceSide;
            case RED123 :
                return this.red123SourceSide;
            case BLUEWING :
                return this.blueWing;
            case BLUEFOURNOTE :
                return this.blueFourNote;
            case BLUEFIVENOTE :
                return this.blueFiveNote;
            case TWONOTE :
                return this.twoNote;
            case REDWING :
                return this.redWing;
            case REDFOURNOTE :
                return this.redFourNote;
            case REDFIVENOTE :
                return this.redFiveNote;
            case DONOTHING :
                return this.doNothing;
        }
        return new InstantCommand();
    }

    /**
     * A method to return the chosen auto command
     * @return command
     */
    public Command getCommand(){
        //this needs to be called here because we might not be connected to the fms before
        subsystems.getCameraSubsystem().setBotPoseSource();
        return new ParallelCommandGroup(
            new ShooterSpinUpAutoCommand(subsystems.getShooterOutfeedSubsystem()),
            new UseFusedVisionInAutoCommand(subsystems.getDriveTrainSubsystem()),
            getAutoPath()
        );
    }

    private Command getBlue123SourceSide(){
        return AutoBuilder.buildAuto("Blue123SourceSide");
    }

    private Command getRed123SourceSide(){
        return AutoBuilder.buildAuto("Red123SourceSide");
    }

    private Command getBlueWing(){
        return AutoBuilder.buildAuto("BlueWing");
    }

    private Command getBlueFourNote(){
        return AutoBuilder.buildAuto("Blue4Note");
    }

    private Command getBlueFiveNote(){
        return AutoBuilder.buildAuto("Blue5Note");
    }

    private Command getTwoNote(){
        return AutoBuilder.buildAuto("TwoNote");
    }

    private Command getRedWing(){
        return AutoBuilder.buildAuto("RedWing");
    }

    private Command getRedFourNote(){
        return AutoBuilder.buildAuto("Red4Note");
    }

    private Command getRedFiveNote(){
        return AutoBuilder.buildAuto("Red5Note");
    }

    private Command getDoNothing(){
        return AutoBuilder.buildAuto("DoNothing");
    }

    private enum AutonomousPath {
        BLUE123,
        RED123,
        BLUEWING,
        BLUEFOURNOTE,
        BLUEFIVENOTE,
        TWONOTE,
        REDWING,
        REDFOURNOTE,
        REDFIVENOTE,
        DONOTHING
    }

    /**
     * configures the PIDs and stuff to be used for autonomous driving
     * @param subsystems
     */
    public static void configureAutoBuilder(SubsystemCollection subsystems){
        HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
          new PIDConstants(1.5, 0.025, 0), // Translation PID constants
          new PIDConstants(4.0, 0.02, 0), // Rotation PID constants
          1.8, // Max module speed, in m/s
          0.43, // Drive base radius in meters. Distance from robot center to furthest module.
          new ReplanningConfig() // Default path replanning config. See the API for the options here
        );

            AutoBuilder.configureHolonomic(
                subsystems.getDriveTrainSubsystem()::getRobotPosition, // Pose supplier
                subsystems.getDriveTrainSubsystem()::setRobotPosition, // Position setter
                subsystems.getDriveTrainSubsystem()::getChassisSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                subsystems.getDriveTrainSubsystem()::drive, // Method that will drive the robot given ROBOT RELATIVE
                                                            // ChassisSpeeds
                pathFollowerConfig,
                () -> false,
            subsystems.getDriveTrainSubsystem());

        if (subsystems.isDriveTrainPowerSubsystemAvailable() &&
            subsystems.isIntakeSubsystemAvailable() && subsystems.isFeederSubsystemAvailable() &&
            subsystems.isShooterAngleSubsystemAvailable() && subsystems.isShooterOutfeedSubsystemAvailable()) {
            NamedCommands.registerCommand("IntakeNote",
                new ParallelCommandGroup(
                    new ButtonPressCommand("PathPlanner", "IntakeNote"),
                    new IntakeAndFeedNoteCommand(subsystems.getIntakeSubsystem(), subsystems.getFeederSubsystem(),
                        FeederMode.FeedToShooter)));
            NamedCommands.registerCommand("OutTakeNote",
                new ParallelCommandGroup(
                    new ButtonPressCommand("PathPlanner", "OuttakeNote"),
                    new RemoveNoteCommand(subsystems.getIntakeSubsystem())));
            NamedCommands.registerCommand("FeedNote",
                new ParallelCommandGroup(
                    new ButtonPressCommand("PathPlanner", "FeedNote"),
                    new FeederLaunchNote(subsystems.getFeederSubsystem(), FeederMode.FeedToShooter, Constants.feederLaunchTimeoutSecondsInAuto)));
            NamedCommands.registerCommand("AngleFromNote",
                new ParallelCommandGroup(
                    new ButtonPressCommand("PathPlanner", "AngleFromNote"),
                    new ShooterSetAngleCommand(39.5, subsystems.getShooterAngleSubsystem())));
            NamedCommands.registerCommand("AngleFromSpeaker",
                new ParallelCommandGroup(
                    new ButtonPressCommand("PathPlanner", "AngleFromSpeaker"),
                    new ShooterSetAngleCommand(56.0, subsystems.getShooterAngleSubsystem())));
            NamedCommands.registerCommand("AngleFromStage",
                new ParallelCommandGroup(
                    new ButtonPressCommand("PathPlanner", "AngleFromStage"),
                    new ShooterSetAngleCommand(38.0, subsystems.getShooterAngleSubsystem())));
            NamedCommands.registerCommand("AngleFromWing",
                new ParallelCommandGroup(
                    new ButtonPressCommand("PathPlanner", "AngleFromWing"),
                    new ShooterSetAngleCommand(30.0, subsystems.getShooterAngleSubsystem())));
            NamedCommands.registerCommand("AngleFromFront",
                new ParallelCommandGroup(
                    new ButtonPressCommand("PathPlanner", "AngleFromFront"),
                    new ShooterSetAngleCommand(40.0, subsystems.getShooterAngleSubsystem())));
            NamedCommands.registerCommand("AngleFromUnder",
                new ParallelCommandGroup(
                    new ButtonPressCommand("PathPlanner", "AngleFromUnder"),
                    new ShooterSetAngleCommand(28.5, subsystems.getShooterAngleSubsystem())));
            NamedCommands.registerCommand("AutoAngle",
                new ParallelCommandGroup(
                    new ButtonPressCommand("PathPlanner", "AutoAngle"),
                    new ShooterSetAngleWithVisionOneShotCommand(subsystems.getCameraSubsystem(), subsystems.getShooterAngleSubsystem())));
        }
    }
}