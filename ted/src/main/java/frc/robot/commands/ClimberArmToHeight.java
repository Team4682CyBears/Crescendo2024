// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: ClimberArmToHeight.java
// Intent: Forms a command to set a climber to a height
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import frc.robot.control.Constants;
import frc.robot.subsystems.ClimberSubsystem;

import edu.wpi.first.wpilibj2.command.Command;

/**
 * Forms a command to move climber to a specific height
 */
public class ClimberArmToHeight extends Command {

  private ClimberSubsystem climber;
  private double requestedHeightInInchesRight = 0.0;
  private double requestedHeightInInchesLeft = 0.0;
  private boolean updatedHeightSet = false;
  private boolean done = false;

  /**
   * Constructor for ClimberArmToHeight
   * Will set shooter to desired angle before shooting
   * @param climberSubsystem - the target climber subsystem
   * @param desiredHeightInInchesLeft - the target updated height of the climber left
   * @param desiredHeightInInchesRight - the target updated height of the climber right
   * @param targetArm - the target climber arm
   */
  public ClimberArmToHeight(
    ClimberSubsystem climberSubsystem, 
    double desiredHeightInInchesLeft,
    double desiredHeightInInchesRight ) {

    this.climber = climberSubsystem;
    this.requestedHeightInInchesLeft = desiredHeightInInchesLeft;
    this.requestedHeightInInchesRight = desiredHeightInInchesRight;
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
      this.climber.setClimberHeightsInInches(this.requestedHeightInInchesLeft, this.requestedHeightInInchesRight);
    }
    else {
      done = this.climber.areClimbersWithinTolerance(Constants.climberStandardToleranceInches);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(!done) {
      // if not done stop the climbers with speed 0
      this.climber.setClimberSpeeds(0.0, 0.0);
    }
    if(interrupted){
      done = true;
      System.out.println("interrupted end of ClimberArmToHeight ... ");
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return done; 
  }

}