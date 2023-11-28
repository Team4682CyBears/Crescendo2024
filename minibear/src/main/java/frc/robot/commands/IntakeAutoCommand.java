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
import frc.robot.Constants;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WristSubsystem;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.common.WristPosition;


public class IntakeAutoCommand extends CommandBase {

    private IntakeSubsystem intakeSubsystem;
    private WristSubsystem wristSubsystem;
    
    private Timer timer = new Timer();
    private boolean done = false;
    private final double shootTimeStampSeconds = 1;

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
        if (timer.hasElapsed(shootTimeStampSeconds) == false) {
            double shotSpeed = this.getExpelSpeedForCurrentWristPosition();
            this.intakeSubsystem.setIntakeRelativeSpeed(shotSpeed);
        }
        else {
            this.intakeSubsystem.setIntakeRelativeSpeed(0.0);
            done = true;      
        }
    }   

    /**
     * Override of the end method on CommandBase - Called once the command ends or is interrupted.
     */
    @Override
    public void end(boolean interrupted) {
        if(interrupted) {
        this.intakeSubsystem.setIntakeRelativeSpeed(0.0);
          done = true;      
        }
      }
    
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return done;
    }

    /**
     * A method that will get the proper expell speed associated with the current robot wrist position
     * @return a double of expell speed that is appropriate for the curren wrist position
     */
    private double getExpelSpeedForCurrentWristPosition() {
        double expelSpeed = Constants.SHOOT_SPEED_3;
        WristPosition currentPosition = wristSubsystem.getTargetWristPosition();
        if(currentPosition == WristPosition.PickUp) {
            expelSpeed = Constants.SHOOT_SPEED_0; // assuming that pickup equates to shot speed 0 - change if this is incorrect assumption!!
        }
        else if(currentPosition == WristPosition.PositionOne) {
            expelSpeed = Constants.SHOOT_SPEED_1; // assuming that position 1 equates to shot speed 2 - change if this is incorrect assumption!!
        }
        else if(currentPosition == WristPosition.PositionTwo) {
            expelSpeed = Constants.SHOOT_SPEED_2; // assuming that position 2 equates to shot speed 2 - change if this is incorrect assumption!!
        }
        else if(currentPosition == WristPosition.PositionThree) {
            expelSpeed = Constants.SHOOT_SPEED_3; // assuming that position 3 equates to shot speed 3 - change if this is incorrect assumption!!
        }
        return expelSpeed;
    }
}