// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: DriveRampDownSpeedCommand.java
// Intent: Forms a command to ramp down the drivetrain subsystem speed until it reaches maximum.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DrivetrainPowerSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class DriveRampDownSpeedCommand extends CommandBase {

    private DrivetrainPowerSubsystem drivetrainPowerSubsystem;
    private Timer timer = new Timer();
    private boolean done = false; 
    private final double powerIncrementDelaySeconds = 0.1;


    public DriveRampDownSpeedCommand(DrivetrainPowerSubsystem drivetrainPowerSub) {
        this.drivetrainPowerSubsystem = drivetrainPowerSub;
        addRequirements(this.drivetrainPowerSubsystem);
    }

    /**
     * Override of the initialize method on CommandBase - called on start of command
     */
    @Override
    public void initialize() {
        done = drivetrainPowerSubsystem.decrementPowerReductionFactor(drivetrainPowerSubsystem.getReducedSpeedReductionFactor());
        timer.reset();
        timer.start();
    }

    /**
     * Override of the execute method on CommandBase - Called every time the scheduler runs while the command is scheduled.
     */
    @Override
    public void execute() {
        // Repeatedly call decrement after every delay until the drivetrain is at the minimum
        if (timer.hasElapsed(this.powerIncrementDelaySeconds)) {
            done = drivetrainPowerSubsystem.decrementPowerReductionFactor(drivetrainPowerSubsystem.getReducedSpeedReductionFactor());
            timer.reset();
            timer.start();
        }
    }   

    /**
     * Override of the end method on CommandBase - Called once the command ends or is interrupted.
     */
    @Override
    public void end(boolean interrupted) {
        if (interrupted){
            done = true; 
        }
    }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return done;
    }
}