// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: AllignWithTag.java
// Intent: Forms a command to allign itself with a designated april tag.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.subsystems.DrivetrainSubsystem;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.common.MotorUtils;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * A class that forms a command to move the robot in the Y direction to
 * center on an april tag.
 */
public class AllignWithTag extends CommandBase {
    private boolean done = false;
    private double designatedTagId;
    private final double velocityValue = 0.3;
    private PIDController yPid = new PIDController(1.0, 0.0, 0.0);
    private DrivetrainSubsystem drivetrainsubsystem = null;
    private final int defaultDoubleArraySize = 6;
    //in the "botpose_targetspace" array the relative bot y is the first value
    private final int indexOfrelativeBotY = 0;
    private final double lateralBotPositionTolerance = 0.05;
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    /**
     * A constructor for the AlignWithTag command.
     * 
     * @param tagID               - the tag ID of the desired april tag
     * @param drivetrainSubsystem - the drivetrain subsystem to be contorlled
     */
    public AllignWithTag(double tagID, DrivetrainSubsystem drivetrainSubsystem) {
        this.designatedTagId = tagID;
        this.drivetrainsubsystem = drivetrainSubsystem;
        addRequirements(drivetrainsubsystem);
    }

    /**
     * A method to inialize the AlignWithAprilTag command
     */
    @Override
    public void initialize() {
        done = false;
    }

    /**
     * A method that runs in every clock cycle
     */
    @Override
    public void execute() {
        // "tid" is the network table entry for the Tag ID
        // using 0.0 as the default value for getDouble so that if the tag isn't read, the robot doesn't move. 
        double tagId = table.getEntry("tid").getDouble(0.0);
        double[] relativeBotpos = table.getEntry("botpose_targetspace").getDoubleArray(new double[defaultDoubleArraySize]);
        double relativeBotY = relativeBotpos[indexOfrelativeBotY];

        if (tagId != designatedTagId) {
            done = true;
        } 
        else {
            double velocity = yPid.calculate(relativeBotY, 0.0);
            velocity = -1 * MotorUtils.clamp(velocity, -velocityValue, velocityValue);
            drivetrainsubsystem.drive(new ChassisSpeeds(0, velocity, 0));
        }
    }

    /**
     * A method to run at the end of the command
     * 
     * @param interrupted - whether the command was interrupted
     */
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            done = true;
            drivetrainsubsystem.drive(new ChassisSpeeds(0,0,0));
        }
    }

    /**
     * A method that returns true when the robot is alligned
     */
    @Override
    public boolean isFinished() {
        double[] relativeBotpos = table.getEntry("botpose_targetspace").getDoubleArray(new double[defaultDoubleArraySize]);
        double relativeBotY = relativeBotpos[indexOfrelativeBotY];
        if (Math.abs(relativeBotY) < lateralBotPositionTolerance) {
            done = true;
        }
        return done;
    }
}
