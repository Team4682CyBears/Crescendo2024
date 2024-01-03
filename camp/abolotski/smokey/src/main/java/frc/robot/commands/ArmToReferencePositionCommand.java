// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: ArmToReferencePositionCommand.java
// Intent: Forms a command to move the dual part arm with y and z inputs.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ArmSubsystem;

/*
 * Forms a command to move the dual part arm with y and z motor inputs.  
 * Y motor input for the extension/horizontal arm.
 * Z motor input for the angle/vertical arm.
 */
public class ArmToReferencePositionCommand extends CommandBase {
    private final ArmSubsystem armSubsystem;
    private Timer timer = new Timer();
    private boolean done = false;
    private final double referenceMaximumTime = 1.0;

    /**
     * The constructor to create a default command for the arm subsystem.
     * @param theArmSubsystem - the arm subsystem
     */
    public ArmToReferencePositionCommand(ArmSubsystem theArmSubsystem) {
        this.armSubsystem = theArmSubsystem;
        addRequirements(this.armSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize()
    {
        timer.reset();
        timer.start();
        done = false;
    }

    @Override
    public void execute() {

        if (armSubsystem.haveArmsFoundSensorReset()) {
            this.done = true;
            System.out.println("Arm to reference: Arms Found Sensor Reset!  Time (seconds) == " + timer.get());
        }
        else if(timer.hasElapsed(this.referenceMaximumTime)) {
            this.done = true;
            System.out.println("Arm to reference: Timer has elapsed!  Time (seconds) == " + timer.get());
        }
        else {
            armSubsystem.moveArmsToSensorReset();
        }
    }

    @Override
    public void end(boolean interrupted) {
        this.armSubsystem.setArmSpeeds(0.0, 0.0);
        if(interrupted) {
            done = true;
            System.out.println("Arm to reference: Interrupted!  Time (seconds) == " + timer.get());
        }
    }
    
    @Override
    public boolean isFinished() {
        return this.done;
    }
}