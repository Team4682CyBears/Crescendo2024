// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: ShooterSubsystem.java
// Intent: Forms the prelminary code for shooter subsystem.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.control.Constants;
import frc.robot.common.Gains;

public class ShooterSubsystem extends SubsystemBase {

  // Talon info
  private TalonFX leftMotor = new TalonFX(Constants.leftTalonShooterMotorCanId);
  private Gains leftMotorVelocityGains = new Gains(5.0, 0.1, 0.001, 1023/20660.0, 300, 1.00);
  private final VelocityVoltage leftMotorVoltageVelocity = new VelocityVoltage(
    0,
    0,
    true,
    0,
    0,
    false,
    false,
    false);
  private Gains leftMotorTorqueGains = new Gains(5.0, 0.1, 0.001, 1023/20660.0, 300, 1.00);
  private final VelocityTorqueCurrentFOC leftMotorTorqueVelocity = new VelocityTorqueCurrentFOC(
    0,
    0,
    0,
    1,
    false,
    false,
    false);
  private TalonFXConfiguration leftMotorConfigs = new TalonFXConfiguration();

  /*
  private TalonFX rightMotor = new TalonFX(Constants.rightTalonShooterMotorCanId);
  private Gains rightMotorGains = new Gains(0.50, 0.001, 5, 1023/20660.0, 300, 1.00);
  private final VelocityTorqueCurrentFOC rightMotorTorqueVelocity = new VelocityTorqueCurrentFOC(
    0,
    0,
    0,
    0,
    false,
    false,
    false);
  private TalonFXConfiguration rightMotorConfigs = new TalonFXConfiguration();
*/


  public ShooterSubsystem() {

    leftMotor.setNeutralMode(NeutralModeValue.Coast);

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    leftMotorConfigs.Slot0.kP = 0.11; // An error of 1 rotation per second results in 2V output
    leftMotorConfigs.Slot0.kI = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
    leftMotorConfigs.Slot0.kD = 0.0001; // A change of 1 rotation per second squared results in 0.01 volts output
    leftMotorConfigs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
    // Peak output of 8 volts
    leftMotorConfigs.Voltage.PeakForwardVoltage = 12;
    leftMotorConfigs.Voltage.PeakReverseVoltage = -12;    

    // Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself
    leftMotorConfigs.Slot1.kP = leftMotorTorqueGains.kP; 
    leftMotorConfigs.Slot1.kI = leftMotorTorqueGains.kI;
    leftMotorConfigs.Slot1.kD = leftMotorTorqueGains.kD;
    // Peak output of 40 amps
    leftMotorConfigs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    leftMotorConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -40;

    leftMotorConfigs.MotorOutput.Inverted = Constants.leftTalonShooterMotorDefaultDirection;

    // Retry config apply up to 5 times, report if failure
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = leftMotor.getConfigurator().apply(leftMotorConfigs);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs to left motor, error code: " + status.toString());
    }

/*
    rightMotor.setNeutralMode(NeutralModeValue.Coast);
    // Torque-based velocity does not require a feed forward, as torque will accelerate the rotor up to the desired velocity by itself
    rightMotorConfigs.Slot0.kP = rightMotorGains.kP; 
    rightMotorConfigs.Slot0.kI = rightMotorGains.kI;
    rightMotorConfigs.Slot0.kD = rightMotorGains.kD;
    // Peak output of 40 amps
    rightMotorConfigs.TorqueCurrent.PeakForwardTorqueCurrent = 40;
    rightMotorConfigs.TorqueCurrent.PeakReverseTorqueCurrent = -40;
    leftMotorConfigs.MotorOutput.Inverted = Constants.rightTalonShooterMotorDefaultDirection;

    // Retry config apply up to 5 times, report if failure
    status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = rightMotor.getConfigurator().apply(rightMotorConfigs);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs to right motor, error code: " + status.toString());
    }
*/
//    CommandScheduler.getInstance().registerSubsystem(this);
  }

  /**
   * Set the top shooter motor to a specific velocity using the in-built PID controller
   * @param revolutionsPerMinute - the RPM that the top motor should spin
   */
  public void setShooterVelocityLeft(double revolutionsPerMinute)
  {
    double desiredRotationsPerSecond = revolutionsPerMinute / 60.0;
    double feedForwardFictionTorque = (desiredRotationsPerSecond > 0) ? 1 : -1;
    System.out.println("trying to run motor, desiredRotationsPerSecond == " + desiredRotationsPerSecond);
    leftMotor.setControl(leftMotorVoltageVelocity.withVelocity(desiredRotationsPerSecond));
//    leftMotor.set(revolutionsPerMinute/6500);
//    leftMotor.setControl(leftMotorTorqueVelocity.withVelocity(desiredRotationsPerSecond).withFeedForward(feedForwardFictionTorque));
  }

  /**
   * Set the top shooter motor to a specific velocity using the in-built PID controller
   * @param revolutionsPerMinute - the RPM that the top motor should spin
   */
/*
  public void setShooterVelocityRight(double revolutionsPerMinute)
  {
    double desiredRotationsPerSecond = revolutionsPerMinute / 60.0;
    double feedForwardFictionTorque = (desiredRotationsPerSecond > 0) ? 1 : -1;
    rightMotor.setControl(rightMotorTorqueVelocity.withVelocity(desiredRotationsPerSecond).withFeedForward(feedForwardFictionTorque));
  }
*/

  public void setAllStop() {
    this.setShooterVelocityLeft(0.0);
//    this.setShooterVelocityRight(0.0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

}