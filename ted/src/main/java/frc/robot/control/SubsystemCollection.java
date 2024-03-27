// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: SubsystemCollection.java
// Intent: Forms a container that stores references to the current subsystems.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.control;
import frc.robot.subsystems.ClimberSubsystem;
import frc.robot.subsystems.DrivetrainPowerSubsystem;
import frc.robot.subsystems.DrivetrainSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.PowerDistributionPanelWatcherSubsystem;
import frc.robot.subsystems.ShooterAngleSubsystem;
import frc.robot.subsystems.ShooterOutfeedSubsystem;
import frc.robot.subsystems.CameraSubsystem;

public class SubsystemCollection
{
    // declaring input classes
    private ManualInputInterfaces manualInput = null;

    // declaring and init subsystems  
    private CameraSubsystem cameraSubsystem = null;
    private ClimberSubsystem climberSubsystem = null;
    private DrivetrainSubsystem driveTrainSubsystem = null;
    private DrivetrainPowerSubsystem driveTrainPowerSubsystem = null;
    private PowerDistributionPanelWatcherSubsystem powerDistributionPanelWatcherSubsystem = null; 
    private ShooterOutfeedSubsystem shooterOutfeedSubsystem = null;
    private ShooterAngleSubsystem shooterAngleSubsystem = null;
    private IntakeSubsystem intakeSubsystem = null;
    private FeederSubsystem feederSubsystem = null;
    private LEDSubsystem LEDSubsystem = null;

    /**
     * Default constructor
     */
    public SubsystemCollection() {}

    public ClimberSubsystem getClimberSubsystem() { return climberSubsystem; }
    public void setClimberSubsystem(ClimberSubsystem value) { climberSubsystem = value; }
    public boolean isClimberSubsystemAvailable() { return climberSubsystem != null; }

    public DrivetrainSubsystem getDriveTrainSubsystem() { return driveTrainSubsystem; }
    public void setDriveTrainSubsystem(DrivetrainSubsystem value) { driveTrainSubsystem = value; }
    public boolean isDriveTrainSubsystemAvailable() { return driveTrainSubsystem != null; }

    public CameraSubsystem getCameraSubsystem() { return cameraSubsystem; }
    public void setCameraSubsystem(CameraSubsystem value) { cameraSubsystem = value; }
    public boolean isCameraSubsystemAvailable() { return cameraSubsystem != null; }

    public LEDSubsystem getLEDSubsystem() { return LEDSubsystem; }
    public void setLEDSubsystem(LEDSubsystem value) { LEDSubsystem = value; }
    public boolean isLEDSubsystemAvailable() { return cameraSubsystem != null; }

    public DrivetrainPowerSubsystem getDriveTrainPowerSubsystem() { return driveTrainPowerSubsystem; }
    public void setDriveTrainPowerSubsystem(DrivetrainPowerSubsystem value) { driveTrainPowerSubsystem = value; }
    public boolean isDriveTrainPowerSubsystemAvailable() { return driveTrainPowerSubsystem != null; }

    public PowerDistributionPanelWatcherSubsystem getPowerDistributionPanelWatcherSubsystem() { return powerDistributionPanelWatcherSubsystem; }
    public void setPowerDistributionPanelWatcherSubsystem(PowerDistributionPanelWatcherSubsystem value) { powerDistributionPanelWatcherSubsystem = value; }
    public boolean isPowerDistributionPanelWatcherSubsystemAvailable() { return powerDistributionPanelWatcherSubsystem != null; }
    
    public ManualInputInterfaces getManualInputInterfaces() { return manualInput; }
    public void setManualInputInterfaces(ManualInputInterfaces value) { manualInput = value; }
    public boolean isManualInputInterfacesAvailable() { return manualInput != null; }

    public IntakeSubsystem getIntakeSubsystem() { return intakeSubsystem; }
    public void setIntakeSubsystem(IntakeSubsystem value) { intakeSubsystem = value; }
    public boolean isIntakeSubsystemAvailable() { return intakeSubsystem != null; }

    public FeederSubsystem getFeederSubsystem() { return feederSubsystem; }
    public void setFeederSubsystem(FeederSubsystem value) { feederSubsystem = value; }
    public boolean isFeederSubsystemAvailable() { return feederSubsystem != null; }

    public ShooterOutfeedSubsystem getShooterOutfeedSubsystem() { return shooterOutfeedSubsystem; }
    public void setShooterOutfeedSubsystem(ShooterOutfeedSubsystem value) { shooterOutfeedSubsystem = value; }
    public boolean isShooterOutfeedSubsystemAvailable() { return shooterOutfeedSubsystem != null; }

    public ShooterAngleSubsystem getShooterAngleSubsystem() { return shooterAngleSubsystem; }
    public void setShooterAngleSubsystem(ShooterAngleSubsystem value) { shooterAngleSubsystem = value; }
    public boolean isShooterAngleSubsystemAvailable() { return shooterAngleSubsystem != null; }
}
