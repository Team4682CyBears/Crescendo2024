// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: RumbleCommand.java
// Intent: Forms a command to make the XBox controller rumble/vibrate.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj.Timer;

/**
 * A class to rumble the xbox controller
 */
public class RumbleCommand extends CommandBase{
    private Timer timer = new Timer();
    private boolean done = false;
    private double rumbleMaximumSeconds = 0.0;

    // NOTE! rumble requires XboxController.  For buttons and axes, we use CommandXboxController
    private XboxController controller = null;
    
    /**
     * constructor for rumble command
     * @param controller
     */
    public RumbleCommand(XboxController controller, double durationSeconds) {  
        this.controller= controller;
        rumbleMaximumSeconds = durationSeconds;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        done = false;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        controller.setRumble(RumbleType.kBothRumble, 1.0);
        if (timer.hasElapsed(rumbleMaximumSeconds)) {
          done = true;
        }
    }   

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        controller.setRumble(RumbleType.kBothRumble, 0.0);
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