// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: DriveFinePlacementCommand.java
// Intent: Forms a command to drive the robot in fine placement mode.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.common.SwerveDriveCenterOfRotation;
import frc.robot.subsystems.DrivetrainSubsystem;

/**
 * A class to drive the robot in fine placement mode
 */
public class DriveFinePlacementCommand extends CommandBase {
    private final DrivetrainSubsystem drivetrainSubsystem;

    private ChassisSpeeds commandedChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    private double rotationalVelocity = 0.0;

    /**
     * Constructor for a command to drive the robot in fine placement mode.
     * @param drivetrainSubsystem
     * @param rotationalVelocity
     */
    public DriveFinePlacementCommand(DrivetrainSubsystem drivetrainSubsystem,
        double rotationalVelocity) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.rotationalVelocity = rotationalVelocity;
        addRequirements(drivetrainSubsystem);
    }


    /**
     *  A method to initialize a command to drive the robot in fine placement mode.
     */
    @Override
    public void initialize() {
        drivetrainSubsystem.setSwerveDriveCenterOfRotation(SwerveDriveCenterOfRotation.RobotFront);
    }

    /**
     * A method that is called every time the scheduler runs while the command is scheduled.
     */
    @Override
    public void execute() {       
        // field-oriented rotational-only movement
        // around center of rotation at front of snout for fine placement
        commandedChassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            0.0,
            0.0,
            rotationalVelocity,
            drivetrainSubsystem.getGyroscopeRotation()
            );   

        drivetrainSubsystem.drive(commandedChassisSpeeds);        
    }

    /**
     * A method that is called when the command completes.
     */
    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        drivetrainSubsystem.setSwerveDriveCenterOfRotation(SwerveDriveCenterOfRotation.RobotCenter);
    }

}