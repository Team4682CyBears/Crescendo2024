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

import frc.robot.commands.AutoShooterSpinUpCommand;
import frc.robot.commands.ButtonPressCommand;
import frc.robot.commands.FeedNoteCommand;
import frc.robot.commands.IntakeAndFeedNoteCommand;
import frc.robot.commands.ShooterShootCommand;
import frc.robot.commands.RemoveNoteCommand;
import frc.robot.commands.ShooterSetAngleCommand;
import frc.robot.commands.ShooterSetAngleWithVisionCommand;
import frc.robot.common.FeederMode;
import frc.robot.commands.FeederLaunchNote;
import frc.robot.control.SubsystemCollection;

public class AutonomousChooser {
    private SubsystemCollection subsystems;
    private final SendableChooser<AutonomousPath> autonomousPathChooser = new SendableChooser<>();

    private Command blue123SourceSide;
    private Command red123SourceSide;
    private Command blueWing;
    private Command blueFourNote;
    private Command blueRush;
    private Command twoNote;

    public AutonomousChooser(SubsystemCollection subsystems){
        this.subsystems = subsystems;

        //TODO make so if we dont have shoot or intake etc we still get mobility
        if (subsystems.isDriveTrainPowerSubsystemAvailable() &&
        subsystems.isIntakeSubsystemAvailable() && subsystems.isFeederSubsystemAvailable() &&
        subsystems.isShooterAngleSubsystemAvailable() && subsystems.isShooterOutfeedSubsystemAvailable()){

        autonomousPathChooser.setDefaultOption("Two Note", AutonomousPath.TWONOTE);
        autonomousPathChooser.addOption("BLUE 123 Source Side", AutonomousPath.BLUE123);
        autonomousPathChooser.addOption("RED 123 Source Side", AutonomousPath.RED123);
        autonomousPathChooser.addOption("BLUE Wing Source Side", AutonomousPath.BLUEWING);
        autonomousPathChooser.addOption("BLUE Four Note", AutonomousPath.BLUEFOURNOTE);
        autonomousPathChooser.addOption("BLUE Rush", AutonomousPath.BLUERUSH);

        SmartDashboard.putData(autonomousPathChooser);

        this.blue123SourceSide = getBlue123SourceSide();
        this.red123SourceSide = getRed123SourceSide();
        this.blueWing = getBlueWing();
        this.blueFourNote = getBlueFourNote();
        this.twoNote = getTwoNote();
        this.blueRush = getBlueRush();
        }
        else{
            System.out.println(">>>>> NO auto routine becuase missing subsystems");
        }
    }

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
            case BLUERUSH :
                return this.blueRush;
            case TWONOTE :
                return this.twoNote;
        }
        return new InstantCommand();
    }

    public Command getCommand(){
        return new ParallelCommandGroup(
            new AutoShooterSpinUpCommand(subsystems.getShooterOutfeedSubsystem()),
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

    private Command getBlueRush(){
        return AutoBuilder.buildAuto("BlueRush");
    }

    private Command getTwoNote(){
        return AutoBuilder.buildAuto("TwoNote");
    }

    private enum AutonomousPath {
        BLUE123,
        RED123,
        BLUEWING,
        BLUEFOURNOTE,
        BLUERUSH,
        TWONOTE
    }

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
            NamedCommands.registerCommand("ShootFromSpeaker",
                new ParallelCommandGroup(
                    new ButtonPressCommand("PathPlanner", "ShootFromSpeaker"),
                    new ShooterShootCommand(
                        Constants.shooterAngleShootFromSpeaker,
                        Constants.shooterOutfeedSpeedForAngleShootFromSpeaker, 
                        Constants.shooterOutfeedSpeedForAngleShootFromSpeaker,
                        subsystems.getShooterOutfeedSubsystem(),
                        subsystems.getShooterAngleSubsystem(),
                        subsystems.getFeederSubsystem())));
            NamedCommands.registerCommand("ShootFromNote",
                new ParallelCommandGroup(
                    new ButtonPressCommand("PathPlanner", "ShootFromNote"),
                    new ShooterShootCommand(
                        Constants.shooterAngleShootFromNote, 
                        Constants.shooterOutfeedSpeedForAngleShootFromNote, 
                        Constants.shooterOutfeedSpeedForAngleShootFromNote,
                        subsystems.getShooterOutfeedSubsystem(),
                        subsystems.getShooterAngleSubsystem(),
                        subsystems.getFeederSubsystem())));
            NamedCommands.registerCommand("ShootFromStage",
                new ParallelCommandGroup(
                    new ButtonPressCommand("PathPlanner", "ShootFromStage"),
                    new ShooterShootCommand(
                        Constants.shooterAngleShootFromStage, 
                        Constants.shooterOutfeedSpeedForAngleShootFromStage, 
                        Constants.shooterOutfeedSpeedForAngleShootFromStage,
                        subsystems.getShooterOutfeedSubsystem(),
                        subsystems.getShooterAngleSubsystem(),
                        subsystems.getFeederSubsystem())));
            NamedCommands.registerCommand("ShootFromSourceWing",
                new ParallelCommandGroup(
                    new ButtonPressCommand("PathPlanner", "ShootFromSourceWing"),
                    new ShooterShootCommand(
                        Constants.shooterAngleShootFromSourceWing, 
                        Constants.shooterOutfeedSpeedForAngleShootFromSourceWing, 
                        Constants.shooterOutfeedSpeedForAngleShootFromSourceWing,
                        subsystems.getShooterOutfeedSubsystem(),
                        subsystems.getShooterAngleSubsystem(),
                        subsystems.getFeederSubsystem())));
            NamedCommands.registerCommand("IntakeNote",
                new ParallelCommandGroup(
                    new ButtonPressCommand("PathPlanner", "IntakeNote"),
                    new IntakeAndFeedNoteCommand(subsystems.getIntakeSubsystem(), subsystems.getFeederSubsystem(),
                        FeederMode.FeedToShooter)));
            NamedCommands.registerCommand("OutTakeNote",
                new ParallelCommandGroup(
                    new ButtonPressCommand("PathPlanner", "OuttakeNote"),
                    new RemoveNoteCommand(subsystems.getIntakeSubsystem())));
            NamedCommands.registerCommand("SpingUpShooter",
                new ParallelCommandGroup(
                    new ButtonPressCommand("PathPlanner", "SpinUpShooter"),
                    new AutoShooterSpinUpCommand(subsystems.getShooterOutfeedSubsystem())));
            NamedCommands.registerCommand("FeedNote",
                new ParallelCommandGroup(
                    new ButtonPressCommand("PathPlanner", "FeedNote"),
                    new FeederLaunchNote(subsystems.getFeederSubsystem(), FeederMode.FeedToShooter)));
            NamedCommands.registerCommand("AngleFromNote",
                new ParallelCommandGroup(
                    new ButtonPressCommand("PathPlanner", "AngleFromNote"),
                    new ShooterSetAngleCommand(40.0, subsystems.getShooterAngleSubsystem())));
            NamedCommands.registerCommand("AngleFromSpeaker",
                new ParallelCommandGroup(
                    new ButtonPressCommand("PathPlanner", "AngleFromSpeaker"),
                    new ShooterSetAngleCommand(56.0, subsystems.getShooterAngleSubsystem())));
            NamedCommands.registerCommand("AngleFromStage",
                new ParallelCommandGroup(
                    new ButtonPressCommand("PathPlanner", "AngleFromNote"),
                    new ShooterSetAngleCommand(38.0, subsystems.getShooterAngleSubsystem())));
            NamedCommands.registerCommand("AngleFromWing",
                new ParallelCommandGroup(
                    new ButtonPressCommand("PathPlanner", "AngleFromWing"),
                    new ShooterSetAngleCommand(30.0, subsystems.getShooterAngleSubsystem())));
            NamedCommands.registerCommand("AngleFromFront",
                new ParallelCommandGroup(
                    new ButtonPressCommand("PathPlanner", "AngleFromFront"),
                    new ShooterSetAngleCommand(40.0, subsystems.getShooterAngleSubsystem())));
            NamedCommands.registerCommand("AutoAngle",
                new ParallelCommandGroup(
                    new ButtonPressCommand("PathPlanner", "AutoAngle"),
                    new ShooterSetAngleWithVisionCommand(subsystems.getCameraSubsystem(), subsystems.getShooterAngleSubsystem())));
        }
    }
}