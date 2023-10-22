// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: EveryBotPickerSubsystem.java
// Intent: Subsystem to model the every bot picker motor and associated subsystem.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import frc.robot.common.*;
import frc.robot.Constants;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * A class intended to model the intake
 */
public class EveryBotPickerSubsystem extends SubsystemBase {

    /* *********************************************************************
    CONSTANTS
    ************************************************************************/
    private int isSrxMotor1Inverted = 1; //set 1 for not inverted, set -1 for inverted
    private int isSrxMotor2Inverted = -1;
    private double requestedSrxMotor1Speed = 0.0;
    private double requestedSrxMotor2Speed = 0.0;
    
    /* *********************************************************************
    MEMBERS
    ************************************************************************/
    private TalonSRX srxMotor1 = new TalonSRX(Constants.SrxMotor1CanId);
    private TalonSRX srxMotor2 = new TalonSRX(Constants.SrxMotor2CanId);


    /* *********************************************************************
    CONSTRUCTORS
    ************************************************************************/

    /**
    * constructor for EveryBotPickerSubsystem subsystem
    */
    public EveryBotPickerSubsystem() {
      CommandScheduler.getInstance().registerSubsystem(this);
    }

    /* *********************************************************************
    PUBLIC METHODS
    ************************************************************************/
    /**
     * A method to set the every bot motor to a certain RPM based on a relative speed input
     * @param everyBotPickerSpeed the relative speed -1.0 to 1.0 to run the everyBot arm motors at
     */
    public void setPickerRelativeSpeed(double srxMotorSpeed) {
      requestedSrxMotor1Speed = MotorUtils.truncateValue(srxMotorSpeed, -1.0, 1.0);
      requestedSrxMotor2Speed = MotorUtils.truncateValue(srxMotorSpeed, -1.0, 1.0);

      if(isCurrentSpikeDetected()) {
        stopMotors();
      } else {
        srxMotor1.set(ControlMode.PercentOutput, requestedSrxMotor1Speed * isSrxMotor1Inverted);
        srxMotor2.set(ControlMode.PercentOutput, requestedSrxMotor2Speed * isSrxMotor2Inverted);
      }

    }

    /**
     * Checks if a spike in current is detected for either SRX motor.
     * @return true if a current spike is detected, false otherwise.
     */
    private boolean isCurrentSpikeDetected() {
      return srxMotor1.getStatorCurrent() > Constants.CURRENT_SPIKE_THRESHOLD || 
             srxMotor2.getStatorCurrent() > Constants.CURRENT_SPIKE_THRESHOLD;
  }
   

  private void stopMotors() {
    srxMotor1.set(ControlMode.PercentOutput, 0);
    srxMotor2.set(ControlMode.PercentOutput, 0);
  }

    /**
     * A method to handle periodic processing
     */
    @Override
    public void periodic() {
      //everyBotMotor.set(this.requestedEveryBotMotorSpeed * this.neoMotorSpeedReductionFactor);
    }

    @Override
    public void simulationPeriodic() {
      // This method will be called once per scheduler run during simulation
    }



}

