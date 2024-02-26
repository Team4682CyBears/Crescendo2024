// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: ClimberArmToHeight.java
// Intent: Forms a command to set a climber to a height
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import frc.robot.common.ClimberArm;
import frc.robot.control.Constants;
import frc.robot.subsystems.ClimberSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * Forms a command to shoot the shooter
 */
public class ClimberArmToHeight extends Command {

  private ClimberSubsystem climber;
  private double requestedHeightInInches = 0.0;
  private ClimberArm requestedArm = ClimberArm.BothClimbers;
  private boolean updatedHeightSet = false;
  private boolean done = false;

  /**
   * Constructor for ShooterShootCommand
   * Will set shooter to desired angle before shooting
   * @param climberSubsystem - the target climber subsystem
   * @param desiredHeightInInches - the target updated height of the climber
   * @param targetArm - the target climber arm
   */
  public ClimberArmToHeight(
    ClimberSubsystem climberSubsystem, 
    ClimberArm targetArm,
    double desiredHeightInInches) {
    this.climber = climberSubsystem;
    this.requestedArm = targetArm;
    this.requestedHeightInInches = desiredHeightInInches;
    // Use addRequirements() here to declare subsystem dependencies.
    this.addRequirements(this.climber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    this.updatedHeightSet = false;
    this.done = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!this.updatedHeightSet) {
        if(requestedArm == ClimberArm.BothClimbers) {
            this.climber.setBothClimberHeightsInInches(this.requestedHeightInInches);
        }
        else if(requestedArm == ClimberArm.RightClimber) {
            this.climber.setRightClimberHeightInInches(this.requestedHeightInInches);
        }
        else if(requestedArm == ClimberArm.LeftClimber) {
            this.climber.setRightClimberHeightInInches(this.requestedHeightInInches);
        }
        else {
            done = true;
        }
    }
    else {
        if(requestedArm == ClimberArm.BothClimbers) {
            done = this.climber.areClimbersWithinTolerance(Constants.climberStandardToleranceInches);
        }
        else if(requestedArm == ClimberArm.RightClimber) {
            done = this.climber.isRightClimberWithinTolerance(Constants.climberStandardToleranceInches);
        }
        else if(requestedArm == ClimberArm.LeftClimber) {
            done = this.climber.isLeftClimberWithinTolerance(Constants.climberStandardToleranceInches);
        }
        else {
            done = true;
        }
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted){
      done = true;
      System.out.println("interrupted end of ClimberArmToPosition ... ");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done; 
  }

}