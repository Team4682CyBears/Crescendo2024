// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: EveryBotPickerAutoUptakeCommand.java
// Intent: Forms a command to automatically uptake the cargo into the every bot picker.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 


package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.common.WristPosition;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;
import edu.wpi.first.wpilibj.Timer;


public class IntakeAutoCommand extends CommandBase {

    private IntakeSubsystem intakeSubsystem;
    private WristSubsystem wristSubsystem;
    
    private Timer timer = new Timer();
    private boolean done = false;
    private final double moveToPositionTimeStamp = 1.7; //TODO set times
    private final double shootTimeStamp = 3;
    private final double shootDoneTimeStamp = 3.5;

    private double relativeSpeed;
    private double durationSeconds;

    /**
     * Constructor for EveryBotPickerAutoUptakeCommand
     * @param intakeSubsystem - the subsystem for the everybot picker
     * @return 
     */
    public IntakeAutoCommand(IntakeSubsystem intakeSubsystem, WristSubsystem wristSubsystem) {
        this.intakeSubsystem = intakeSubsystem;
        addRequirements(this.intakeSubsystem);

        this.wristSubsystem = wristSubsystem;
        addRequirements(this.wristSubsystem);
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
        if (timer.hasElapsed(moveToPositionTimeStamp)) {
            new WristPositionCommand(wristSubsystem, WristPosition.PositionTwo);
        } 

        if (timer.hasElapsed(shootTimeStamp)) {
            new IntakeDefaultCommand(wristSubsystem,
            intakeSubsystem, 
                    () -> 0.0, // No uptake
                    () -> 1.0); // Assuming full power expelling
        }

        if (timer.hasElapsed(shootDoneTimeStamp)) {
            new IntakeDefaultCommand(wristSubsystem,
            intakeSubsystem, 
                    () -> 0.0, // No uptake
                    () -> 0.0); // No uptake

        new WristPositionCommand(wristSubsystem, WristPosition.PositionThree);

        }

        done = true;
    }   

    /**
     * Override of the end method on CommandBase - Called once the command ends or is interrupted.
     */
    @Override
    public void end(boolean interrupted) {
        if(interrupted) {
            new IntakeDefaultCommand(wristSubsystem,
            intakeSubsystem, 
                    () -> 0.0, 
                    () -> 0.0); 
        
        new WristPositionCommand(wristSubsystem, WristPosition.PositionThree);

            
          done = true;      
        }
      }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return done;
    }
}