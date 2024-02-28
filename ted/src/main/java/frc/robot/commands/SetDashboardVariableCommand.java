// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: SetDashboardVariableCommand
// Intent: Forms a command to set a variable on the dashboard to 
// two different values separated by a delay.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * A class to rumble the xbox controller
 */
public class SetDashboardVariableCommand extends Command{
    private Timer timer = new Timer();
    private boolean done = false;
    private double delaySeconds; 
    private double firstValue;
    private double secondValue;
    private String variableName; 
    
    /**
     * constructor for rumble command
     * @param firstValue
     * @param delay
     * @param secondValue
     */
    public SetDashboardVariableCommand(String variableName, double firstValue, double delaySeconds, double secondValue) {  
      this.variableName = variableName;
      this.firstValue = firstValue;
      this.delaySeconds = delaySeconds;
      this.secondValue = secondValue;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        done = false;
        SmartDashboard.putNumber(variableName, firstValue);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (timer.hasElapsed(delaySeconds)) {
          SmartDashboard.putNumber(variableName, secondValue);
          done = true;
        }
    }   

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
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