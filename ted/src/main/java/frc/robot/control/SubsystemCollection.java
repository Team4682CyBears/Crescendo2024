// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: SubsystemCollection.java
// Intent: Forms a container that stores references to the current subsystems.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.control;
import frc.robot.subsystems.*;

public class SubsystemCollection
{
    // declaring input classes
    private ManualInputInterfaces manualInput = null;

    // declaring and init subsystems  
    private DrivetrainSubsystem driveTrainSubsystem = null;
    private CameraSubsystem cameraSubsystem = null;
    private DrivetrainPowerSubsystem driveTrainPowerSubsystem = null;
    private PowerDistributionPanelWatcherSubsystem powerDistributionPanelWatcherSubsystem = null; 
    private TalonShooterSubsystem shooterSubsystem = null;
    private SteerMotorSubsystem steerMotorSubsystem = null;
    private IntakeSubsystem intakeSubsystem = null;
    private FeederSubsystem feederSubsystem = null;
    private SteerMotorCanCoderSubsystem steerMotorCanCoderSubsystem = null;

    /**
     * Default constructor
     */
    public SubsystemCollection() {}

    public DrivetrainSubsystem getDriveTrainSubsystem() { return driveTrainSubsystem; }
    public void setDriveTrainSubsystem(DrivetrainSubsystem value) { driveTrainSubsystem = value; }

    public CameraSubsystem getCameraSubsystem() { return cameraSubsystem; }
    public void setCameraSubsystem(CameraSubsystem value) { cameraSubsystem = value; }

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

    public TalonShooterSubsystem getShooterSubsystem() { return shooterSubsystem; }
    public void setShooterSubsystem(TalonShooterSubsystem value) { shooterSubsystem = value; }

    public SteerMotorSubsystem getSteerMotorSubsystem() { return steerMotorSubsystem; }
    public void setSteerMotorSubsystem(SteerMotorSubsystem value) { steerMotorSubsystem = value; }

    public SteerMotorCanCoderSubsystem getSteerMotorCanCoderSubsystem() { return steerMotorCanCoderSubsystem; }
    public void setSteerMotorCanCoderSubsystem(SteerMotorCanCoderSubsystem value) { steerMotorCanCoderSubsystem = value; }
}
