// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: EveryBotPickerOverCurrentCommand.java
// Intent: Forms a command to automatically make the every bot motor go to zero.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.EveryBotPickerSubsystem;

import edu.wpi.first.wpilibj.Timer;

public class EveryBotPickerOverCurrentCommand extends CommandBase {

    private EveryBotPickerSubsystem everyBotPickerSub;
    private Timer timer = new Timer();
    private boolean done = false;
    private double zeroMotorDurationSeconds = 0.1;

    /**
     * Ccnstructor for EveryBotPickerAutoUptakeCommand
     * @param everyBotPickerSubsystem - the subsystem for the everybot picker
     * @param zeroDurationSeconds - the duration of the motor to stop it
     */
    public EveryBotPickerOverCurrentCommand(
        EveryBotPickerSubsystem everyBotPickerSubsystem,
        double zeroDurationSeconds) {
        this.everyBotPickerSub = everyBotPickerSubsystem;
        this.zeroMotorDurationSeconds = zeroDurationSeconds;
        addRequirements(this.everyBotPickerSub);
    }

    /**
     * Override of the initialize method on CommandBase - called on start of command
     */
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        done = false;
    }

    /**
     * Override of the execute method on CommandBase - Called every time the scheduler runs while the command is scheduled.
     */
    @Override
    public void execute() {
        everyBotPickerSub.setPickerRelativeSpeed(0.0);
        if (timer.hasElapsed(this.zeroMotorDurationSeconds)) {
            done = true;
        }
    }   

    /**
     * Override of the end method on CommandBase - Called once the command ends or is interrupted.
     */
    @Override
    public void end(boolean interrupted) {
        everyBotPickerSub.setPickerRelativeSpeed(0.0);
        if(interrupted) {
          done = true;      
        }
      }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return done;
    }
}