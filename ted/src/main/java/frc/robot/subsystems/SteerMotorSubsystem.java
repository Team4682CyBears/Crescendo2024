// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: SteerMotorSubsystem.java
// Intent: Forms the prelminary code for shooter subsystem.
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.subsystems;

import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.control.Constants;
import frc.robot.common.Gains;

public class SteerMotorSubsystem extends SubsystemBase {

  // Talon info
  private TalonFX steerMotor = new TalonFX(Constants.FRONT_LEFT_MODULE_STEER_MOTOR);
  private final PositionVoltage steerMotorPositionVoltage = new PositionVoltage(
    0,
    0,
    true,
    0,
    0,
    false,
    false,
    false);
  private final NeutralOut brake = new NeutralOut();
  private TalonFXConfiguration steerMotorConfigs = new TalonFXConfiguration();
  private final double steerMotorGearRatio = 396 / 35;
  // the steer motor units per degree of rotation of the steer wheel
  private final double steerMotorDegreesToMotorUnitsFactor = (2048 * this.steerMotorGearRatio) / 360.0;
  private double steerMotorPositionUnits = 0;
  private final double toleranceDelta = 10.0;

  public SteerMotorSubsystem() {

    steerMotor.setNeutralMode(NeutralModeValue.Brake);
    steerMotor.setPosition(steerMotorPositionUnits);

    /* Voltage-based velocity requires a feed forward to account for the back-emf of the motor */
    steerMotorConfigs.Slot0.kP = 2.4; // An error of 0.5 rotations results in 1.2 volts output
    steerMotorConfigs.Slot0.kI = 0.5; // An error of 1 rotation per second increases output by 0.5V every second
    steerMotorConfigs.Slot0.kD = 0.1; // A change of 1 rotation per second results in 0.1 volts output
    steerMotorConfigs.Slot0.kV = 0.12; // Falcon 500 is a 500kV motor, 500rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / Rotation per second
    // Peak output of 8 volts
    steerMotorConfigs.Voltage.PeakForwardVoltage = 12;
    steerMotorConfigs.Voltage.PeakReverseVoltage = -12;    

    steerMotorConfigs.MotorOutput.Inverted = Constants.leftTalonShooterMotorDefaultDirection;

    // Retry config apply up to 5 times, report if failure
    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for (int i = 0; i < 5; ++i) {
      status = steerMotor.getConfigurator().apply(steerMotorConfigs);
      if (status.isOK()) break;
    }
    if(!status.isOK()) {
      System.out.println("Could not apply configs to left motor, error code: " + status.toString());
    }
  }

  /**
   * Set the top shooter motor to a specific velocity using the in-built PID controller
   * @param revolutionsPerMinute - the RPM that the top motor should spin
   */
  public void setSteerMotorAngle(double angle)
  {
    System.out.println("trying to set motor angle to == " + angle);
    this.setSteerMotorPosition(angle * this.steerMotorDegreesToMotorUnitsFactor);
  }

  /**
   * Set the top shooter motor to a specific velocity using the in-built PID controller
   * @param revolutionsPerMinute - the RPM that the top motor should spin
   */
  public void setSteerMotorPosition(double motorPositionUnits)
  {
    System.out.println("trying to set motor position to == " + motorPositionUnits);
    this.steerMotorPositionUnits = motorPositionUnits;
  }

  /**
   * Gets the steer motor position in degrees
   * @return double for the steer motor position in motor units
   */
  public double getSteerMotorDegrees() {
    return this.getSteerMotorPosition() / this.steerMotorDegreesToMotorUnitsFactor;
  }

  /**
   * Gets the steer motor position in degrees
   * @return double for the steer motor position in motor units
   */
  public double getSteerMotorPosition() {
    return this.steerMotor.getPosition().getValueAsDouble();
  }

  /**
   * A function to determine if the motor is at its previously set target position
   * @return - true if at position else false
   */
  public boolean isAtTargetPosition() {
    // This method will be called once per scheduler run
    double currentPositionUnits = this.getSteerMotorPosition();
    double delta = Math.abs(currentPositionUnits - steerMotorPositionUnits);
    return (delta <= this.toleranceDelta);
  }

  @Override
  public void periodic() {
    if(this.isAtTargetPosition()) {
        steerMotor.setControl(brake);
    }
    else {
        steerMotor.setControl(steerMotorPositionVoltage.withPosition(steerMotorPositionUnits));
    }
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}