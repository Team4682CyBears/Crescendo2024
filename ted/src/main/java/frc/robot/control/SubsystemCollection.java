// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: SubsystemCollection.java
// Intent: Forms a container that stores references to the current subsystems.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.control;
import frc.robot.subsystems.DrivetrainPowerSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.PowerDistributionPanelWatcherSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SteerMotorSubsystem;

public class SubsystemCollection
{
    // declaring input classes
    private ManualInputInterfaces manualInput = null;

    // declaring and init subsystems  
    private DrivetrainSubsystem driveTrainSubsystem = null;
    private DrivetrainPowerSubsystem driveTrainPowerSubsystem = null;
    private PowerDistributionPanelWatcherSubsystem powerDistributionPanelWatcherSubsystem = null; 
    private ShooterSubsystem shooterSubsystem = null;
    private SteerMotorSubsystem steerMotorSubsystem = null;
    private IntakeSubsystem intakeSubsystem = null;
    private FeederSubsystem feederSubsystem = null;

    /**
     * Default constructor
     */
    public SubsystemCollection() {}

    public DrivetrainSubsystem getDriveTrainSubsystem() { return driveTrainSubsystem; }
    public void setDriveTrainSubsystem(DrivetrainSubsystem value) { driveTrainSubsystem = value; }

    public DrivetrainPowerSubsystem getDriveTrainPowerSubsystem() { return driveTrainPowerSubsystem; }
    public void setDriveTrainPowerSubsystem(DrivetrainPowerSubsystem value) { driveTrainPowerSubsystem = value; }

    public PowerDistributionPanelWatcherSubsystem getPowerDistributionPanelWatcherSubsystem() { return powerDistributionPanelWatcherSubsystem; }
    public void setPowerDistributionPanelWatcherSubsystem(PowerDistributionPanelWatcherSubsystem value) { powerDistributionPanelWatcherSubsystem = value; }
    
    public ManualInputInterfaces getManualInputInterfaces() { return manualInput; }
    public void setManualInputInterfaces(ManualInputInterfaces value) { manualInput = value; }

    public IntakeSubsystem getIntakeSubsystem() { return intakeSubsystem; }
    public void setIntakeSubsystem(IntakeSubsystem value) { intakeSubsystem = value; }

    public FeederSubsystem getFeederSubsystem() { return feederSubsystem; }
    public void setFeederSubsystem(FeederSubsystem value) { feederSubsystem = value; }

    public ShooterSubsystem getShooterSubsystem() { return shooterSubsystem; }
    public void setShooterSubsystem(ShooterSubsystem value) { shooterSubsystem = value; }

    public SteerMotorSubsystem getSteerMotorSubsystem() { return steerMotorSubsystem; }
    public void setSteerMotorSubsystem(SteerMotorSubsystem value) { steerMotorSubsystem = value; }
}
