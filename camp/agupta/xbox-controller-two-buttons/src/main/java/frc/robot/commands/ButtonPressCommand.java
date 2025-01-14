// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: ButtonPressCommand.java
// Intent: Forms a manual command to print the button number.
// ************************************************************

// place class in local java package
package frc.robot.commands;

// ***  Import frc packages
import edu.wpi.first.wpilibj2.command.Command;

// ***  Import java packages
// project code
import java.util.ArrayDeque;


public class ButtonPressCommand extends Command
{
    /**
     * The constructor 
     */
    public ButtonPressCommand(
        String inputDeviceDescription,
        String inputActionDescription)
    {}

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // Retract the intake with the Y button
        //m_driverController.y().onTrue(m_intake.retractCommand());
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {}

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

}

