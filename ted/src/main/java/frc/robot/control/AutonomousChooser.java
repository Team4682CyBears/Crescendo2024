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

import frc.robot.commands.ButtonPressCommand;
import frc.robot.commands.IntakeAndFeedNoteCommand;
import frc.robot.commands.ShooterShootCommand;
import frc.robot.common.FeederMode;
import frc.robot.control.SubsystemCollection;

public class AutonomousChooser {
    private SubsystemCollection subsystems;
    private final SendableChooser<AutonomousPath> autonomousPathChooser = new SendableChooser<>();

    private Command blue123SourceSide;
    private Command red123SourceSide;
    private Command blueWing;
    private Command redWing;
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
        autonomousPathChooser.addOption("RED Wing Source Side", AutonomousPath.REDWING);

        SmartDashboard.putData(autonomousPathChooser);

        this.blue123SourceSide = getBlue123SourceSide();
        this.red123SourceSide = getRed123SourceSide();
        this.blueWing = getBlueWing();
        this.redWing = getRedWing();
        this.twoNote = getTwoNote();
        }
        else{
            System.out.println(">>>>> NO auto routine becuase missing subsystems");
        }
    }

    public Command getCommand() {
        switch (autonomousPathChooser.getSelected()) {
            case BLUE123 :
                return this.blue123SourceSide;
            case RED123 :
                return this.red123SourceSide;
            case BLUEWING :
                return this.blueWing;
            case REDWING :
                return this.redWing;
            case TWONOTE :
                return this.twoNote;
        }
        return new InstantCommand();
    }

    private Command getBlue123SourceSide(){
        return AutoBuilder.buildAuto("Blue123SourceSide");
    }

    private Command getRed123SourceSide(){
        return AutoBuilder.buildAuto("Red123SourceSide");
    }

    private Command getBlueWing(){
        return AutoBuilder.buildAuto("BlueSourceSideWing");
    }

    private Command getRedWing(){
        return AutoBuilder.buildAuto("RedSourceSideWing");
    }

    private Command getTwoNote(){
        return AutoBuilder.buildAuto("TwoNote");
    }

    private enum AutonomousPath {
        BLUE123,
        RED123,
        BLUEWING,
        REDWING,
        TWONOTE
    }

    public static void configureAutoBuilder(SubsystemCollection subsystems){
        HolonomicPathFollowerConfig pathFollowerConfig = new HolonomicPathFollowerConfig(
          new PIDConstants(2.0, 0, 0), // Translation PID constants
          new PIDConstants(4.5, 0.001, 0), // Rotation PID constants
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
                    new ShooterShootCommand(56.0, 4000.0, 4000.0, subsystems.getShooterOutfeedSubsystem(),
                        subsystems.getShooterAngleSubsystem(), subsystems.getFeederSubsystem())));
            NamedCommands.registerCommand("ShootFromNote",
                new ParallelCommandGroup(
                    new ButtonPressCommand("PathPlanner", "ShootFromNote"),
                    new ShooterShootCommand(42.0, 6000.0, 6000.0, subsystems.getShooterOutfeedSubsystem(),
                        subsystems.getShooterAngleSubsystem(), subsystems.getFeederSubsystem())));
            NamedCommands.registerCommand("ShootFromStage",
                new ParallelCommandGroup(
                    new ButtonPressCommand("PathPlanner", "ShootFromStage"),
                    new ShooterShootCommand(39.0, 6000.0, 6000.0, subsystems.getShooterOutfeedSubsystem(),
                        subsystems.getShooterAngleSubsystem(), subsystems.getFeederSubsystem())));
            NamedCommands.registerCommand("ShootFromSourceWing",
                new ParallelCommandGroup(
                    new ButtonPressCommand("PathPlanner", "ShootFromSourceWing"),
                    new ShooterShootCommand(22.0, 6500.0, 6500.0, subsystems.getShooterOutfeedSubsystem(),
                        subsystems.getShooterAngleSubsystem(), subsystems.getFeederSubsystem())));
            NamedCommands.registerCommand("IntakeNote",
                new ParallelCommandGroup(
                    new ButtonPressCommand("PathPlanner", "IntakeNote"),
                    new IntakeAndFeedNoteCommand(subsystems.getIntakeSubsystem(), subsystems.getFeederSubsystem(),
                        FeederMode.FeedToShooter)));
        }
    }
}
