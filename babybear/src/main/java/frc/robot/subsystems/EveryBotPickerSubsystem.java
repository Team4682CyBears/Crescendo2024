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

import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * A class intended to model the intake
 */
public class EveryBotPickerSubsystem extends SubsystemBase {

    /* *********************************************************************
    CONSTANTS
    ************************************************************************/

    /* *********************************************************************
    MEMBERS
    ************************************************************************/
    private TalonSRX srxMotor1 = new TalonSRX(Constants.SrxMotor1CanId);
    private TalonSRX srxMotor2 = new TalonSRX(Constants.SrxMotor2CanId);
    private boolean isSrxMotor1Inverted = true;
    private boolean isSrxMotor2Inverted = true;
    private double requestedSrxMotor1Speed = 0.0;
    private double requestedSrxMotor2Speed = 0.0;

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
     * @param everyBotPickerSpeed the relative speed -1.0 to 1.0 to run the everyBot arm motor at
     */
    public void setPickerRelativeSpeed(double srxMotorSpeed) {
      this.requestedSrxMotor1Speed = MotorUtils.truncateValue(srxMotorSpeed, -1.0, 1.0);
      this.requestedSrxMotor2Speed = MotorUtils.truncateValue(srxMotorSpeed, -1.0, 1.0);
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

