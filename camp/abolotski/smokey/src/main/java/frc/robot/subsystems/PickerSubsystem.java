// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Charged Up - 2023
// File: PickerSubsystem.java
// Intent: Subsystem to model the picker pneumatics subsystem.
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
 * A class intended to model the picker - checked with Simeon on 02/15/2023 name is now 'picker'
 */
public class PickerSubsystem extends SubsystemBase{

    private boolean currentHorizontalPickerDeployed = false;
    private boolean currentVerticalPickerDeployed = false;
    Compressor compressor = new Compressor(1, Constants.PneumaticsControlModuleType);
    DoubleSolenoid horizontalSolenoid = new DoubleSolenoid(
        Constants.PneumaticsControlModuleNumber,
        Constants.PneumaticsControlModuleType,
        Constants.PickerHorizontalPneumaticsControlModuleForwardChannel,
        Constants.PickerHorizontalPneumaticsControlModuleReverseChannel);
    DoubleSolenoid verticalSolenoid = new DoubleSolenoid(
      Constants.PneumaticsControlModuleNumber,
      Constants.PneumaticsControlModuleType,
      Constants.PickerVerticalPneumaticsControlModuleForwardChannel,
      Constants.PickerVerticalPneumaticsControlModuleReverseChannel);
    /**
     * No argument constructor for the BallHandler subsystem.
    */
    public PickerSubsystem() {
      this.intitalizePickerState();
      CommandScheduler.getInstance().registerSubsystem(this);
    }

    /**
     * Method to obtain if the Horizontal picker is in the retracted position.
     * @return boolean true if Horizontal picker is retracted, otherwise false.
     */
    public boolean isHorizontalDeployed()
    {
        return currentHorizontalPickerDeployed;
    }

    /**
     * Method to obtain if the Horizontal picker is in the retracted position.
     * @return boolean true if Horizontal picker is retracted, otherwise false.
     */
    public boolean isHorizontalRetracted()
    {
        return !currentHorizontalPickerDeployed;
    }

    /**
     * Method to move the Horizontal picker into a deployed (closed) position
     */
    public void deployHorizontalPosition(){
        this.horizontalSolenoid.set(DoubleSolenoid.Value.kForward);
        currentHorizontalPickerDeployed = true;
    }

    /**
     * Method to move the Horizontal picker into a retracted (open) position
     */
    public void retractHorizontalPosition(){
        this.horizontalSolenoid.set(DoubleSolenoid.Value.kReverse);
        currentHorizontalPickerDeployed = false;
    }

    /**
     * Method to toggle the Horizontal picker position from its current location.
     */
    public void toggleHorizontalPosition(){        
        if(currentHorizontalPickerDeployed)
        {
            this.retractHorizontalPosition();
        }
        else
        {
            this.deployHorizontalPosition();
        }
    }

    /**
     * Method to obtain if the Vertical picker is in the retracted position.
     * @return boolean true if Vertical picker is retracted, otherwise false.
     */
    public boolean isVerticalDeployed()
    {
        return currentVerticalPickerDeployed;
    }

    /**
     * Method to obtain if the Vertical picker is in the retracted position.
     * @return boolean true if Vertical picker is retracted, otherwise false.
     */
    public boolean isVerticalRetracted()
    {
        return !currentVerticalPickerDeployed;
    }

    /**
     * Method to move the Vertical picker into a deployed (upward) position
     */
    public void deployVerticalPosition(){
        this.verticalSolenoid.set(DoubleSolenoid.Value.kForward);
        currentVerticalPickerDeployed = true;
    }

    /**
     * Method to move the Vertical picker into a retracted (downward) position
     */
    public void retractVerticalPosition(){
        this.verticalSolenoid.set(DoubleSolenoid.Value.kReverse);
        currentVerticalPickerDeployed = false;
    }

    /**
     * Method to toggle the Vertical picker position from its current location.
     */
    public void toggleVerticalPosition(){        
        if(currentVerticalPickerDeployed)
        {
            this.retractVerticalPosition();
        }
        else
        {
            this.deployVerticalPosition();
        }
    }

    /**
     * A method to handle periodic processing
     */
    @Override
    public void periodic() {
        SmartDashboard.putBoolean("HorizontalPickerDeployed", this.isHorizontalDeployed());
        SmartDashboard.putBoolean("VerticalPickerDeployed", this.isVerticalDeployed());
    }
 
    private void intitalizePickerState()
    {
        // confirm that the double horizontalSolenoid has retracted the arm
        this.retractHorizontalPosition();
        // confirm that the double verticalSolenoid has retracted the arm
        this.retractVerticalPosition();
    }

}