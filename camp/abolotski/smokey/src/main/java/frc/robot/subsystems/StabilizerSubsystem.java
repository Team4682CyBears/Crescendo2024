// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: StabilizerSubsystem.java
// Intent: Subsystem to model the front pneumatics Stabilizer.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * A class intended to model the BallHandler infrastructure on Veer.
 */
public class StabilizerSubsystem extends SubsystemBase{

    private boolean currentStabilizerDeployed = false;
    Compressor compressor = new Compressor(1, Constants.PneumaticsControlModuleType);
    DoubleSolenoid solenoid = new DoubleSolenoid(
        Constants.PneumaticsControlModuleNumber,
        Constants.PneumaticsControlModuleType,
        Constants.StabilizerPneumaticsControlModuleForwardChannel,
        Constants.StabilizerPneumaticsControlModuleReverseChannel);

    /**
     * No argument constructor for the BallHandler subsystem.
    */
    public StabilizerSubsystem() {
        this.intitalizeStabilizerState();
        CommandScheduler.getInstance().registerSubsystem(this);
    }

    /**
     * Method to obtain if the stabilizer is in the retracted position.
     * @return boolean true if stabilizer is retracted, otherwise false.
     */
    public boolean isDeployed()
    {
        return currentStabilizerDeployed;
    }

    /**
     * Method to obtain if the stabilizer is in the retracted position.
     * @return boolean true if stabilizer is retracted, otherwise false.
     */
    public boolean isRetracted()
    {
        return !currentStabilizerDeployed;
    }

    /**
     * Method to move the stabilizer into a deployed (downward) position
     */
    public void deployPosition(){
        this.solenoid.set(DoubleSolenoid.Value.kForward);
        currentStabilizerDeployed = true;
    }

    /**
     * Method to move the stabilizer into a retracted (upward) position
     */
    public void retractPosition(){
        this.solenoid.set(DoubleSolenoid.Value.kReverse);
        currentStabilizerDeployed = false;
    }

    /**
     * Method to toggle the arm position from its current location.
     */
    public void togglePosition(){        
        if(currentStabilizerDeployed)
        {
            this.retractPosition();
        }
        else
        {
            this.deployPosition();
        }
    }

    /**
     * A method to handle periodic processing
     */
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("StabilizerArmDeployed", this.isDeployed());
    }
 
    private void intitalizeStabilizerState()
    {
        // confirm that the double solenoid has retracted the arm
        this.retractPosition();
    }

}