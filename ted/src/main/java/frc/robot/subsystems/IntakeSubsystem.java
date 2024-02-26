// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: IntakeSubsystem.java
// Intent: Forms the prelminary code for intake subsystem.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import java.util.Random;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVLibError;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.common.NoteTofSensor;
import frc.robot.control.Constants;
import frc.robot.control.InstalledHardware;

/**
 * Forms a class for the intake subsystem consisting of
 * a Neo motor and a TOF sensor
 */
public class IntakeSubsystem extends SubsystemBase {
  //Neo motor
  private CANSparkMax intakeMotor = null;
  //BeamBreak sensor
  private NoteTofSensor beambreakSensor = null;
  private int maximumInitTries = 10;
  private int minimumInitWaitDuration = 10;
  private int maximumInitWaitDuration = 100;
  private int defaultInitWaitDuration = 500;
  private Random randy = new Random();

  /**
   * Constructor for the IntakeSubsystem
   */
  public IntakeSubsystem() {
    // move the motor creation to the constructor, since it was erroring when done as a class variable. 
    for(int inx = 0; inx < this.maximumInitTries && intakeMotor == null; ++inx) {
      System.out.println("Initializing Intake Motor try# " + inx+1 + " ...");
      boolean allIsWell = true;
      try {
        CANSparkMax assembledMotor = new CANSparkMax(Constants.intakeMotorCanId, MotorType.kBrushless);
        allIsWell &= this.confirmLastErrorOnMotor(assembledMotor);
        assembledMotor.setIdleMode(IdleMode.kBrake);
        allIsWell &= this.confirmLastErrorOnMotor(assembledMotor);
        assembledMotor.set(0.0);
        allIsWell &= this.confirmLastErrorOnMotor(assembledMotor);

        if(allIsWell){
          intakeMotor = assembledMotor;
          int loopCount = this.defaultInitWaitDuration / this.minimumInitWaitDuration;
          for(int jnx = 0; jnx < loopCount; ++jnx) {
            Thread.sleep(this.minimumInitWaitDuration);
            System.out.print(".");
          }
        }
        else{
          int randomWaitMilliseconds = randy.nextInt(this.minimumInitWaitDuration, this.maximumInitWaitDuration);
          Thread.sleep(randomWaitMilliseconds);
        }
      }
      catch(Exception ex){
        System.out.println(ex.toString());
      }
    }
    if (InstalledHardware.intakeTofInstalled){
      beambreakSensor = new NoteTofSensor(Constants.intakeTofCanId);
      beambreakSensor.setDisplayName("Intake TOF");
    }
  }

  /**
   * A method to detect the presence of a note
   * @return true if the note is detected
   */
  public boolean isNoteDetected(){
    if (beambreakSensor != null){
      return beambreakSensor.isNoteDetected();
    }
    return false;
  }

  /**
   * periodic for this subsystem. Called once per scheduler run (20ms) 
   */
  @Override
  public void periodic() {
    if(this.beambreakSensor != null) {
      this.beambreakSensor.publishTelemetery();
    }
  }

  /**
   * A method to set the intake speed
   * @param speed (between -1 and 1)
   */
  public void setIntakeSpeed(double speed) {
    intakeMotor.set(speed);
  }

  /**
   * A method to stop the intake subsystem
   */
  public void setAllStop() {
    intakeMotor.set(0.0);
  }

  /**
   * This method will be called once per scheduler run during simulation
   */
  @Override
  public void simulationPeriodic() {
  }

  /**
   * Method to test last motor error of success or failure
   * @param motor - the instance of a CANSparkMax motor that we are testing
   * @return true if all is good / last error kOK
   */
  private boolean confirmLastErrorOnMotor(CANSparkMax motor) {
    REVLibError error = motor.getLastError();
    boolean result = (error == REVLibError.kOk);
    if(!result){
      System.out.println("Not OK motor last error == " + motor.getLastError());    
    }
    return result;
  }
}