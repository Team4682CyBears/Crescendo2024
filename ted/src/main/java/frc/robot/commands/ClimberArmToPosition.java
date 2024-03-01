// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: ClimberArmToPosition.java
// Intent: Forms a command to set a climber to a position
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.commands;

import frc.robot.common.ClimberArm;
import frc.robot.common.ClimberArmTargetPosition;
import frc.robot.subsystems.ClimberSubsystem;

/**
 * Forms a command to shoot the shooter
 */
public class ClimberArmToPosition extends ClimberArmToHeight {

  /**
   * Constructor for ShooterShootCommand
   * Will set shooter to desired angle before shooting
   * @param climberSubsystem - the target climber subsystem
   * @param targetArm - the target climber arm
   * @param ClimberArmTargetPosition - the target updated position for the climber
   */
  public ClimberArmToPosition(
    ClimberSubsystem climberSubsystem,
    ClimberArm targetArm,
    ClimberArmTargetPosition targetPosition) {
        super(
            climberSubsystem,
            ClimberArmToPosition.getHeightInInchesForNamedPosition(targetPosition),
            ClimberArmToPosition.getHeightInInchesForNamedPosition(targetPosition));
  }

  private static double getHeightInInchesForNamedPosition(ClimberArmTargetPosition position) {
    double targetHeight = 0.0;
    if(position == ClimberArmTargetPosition.FullDeploy) {
        targetHeight = 19.5;
    }
    else if(position == ClimberArmTargetPosition.FullRetract) {
        targetHeight = 0.0;
    }
    else if(position == ClimberArmTargetPosition.HighChain) {
        targetHeight = 18.0;
    }
    else if(position == ClimberArmTargetPosition.LowChain) {
        targetHeight = 15.0;
    }
    else if(position == ClimberArmTargetPosition.HangRobot) {
        targetHeight = 10.0;
    }
    return targetHeight;
  }
}