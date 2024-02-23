// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: SteerMotorCanCoderSubsystem.java
// Intent: Same name extension 
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 
package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.control.Constants;
import frc.robot.swerveLib.ctre.CtreUtils;

public class SteerMotorCanCoderSubsystem extends SubsystemBase {

    private CANcoder encoder = new CANcoder(Constants.FRONT_LEFT_MODULE_STEER_ENCODER);
    private CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
    private int counter = 0;
    private double encoderOffsetDegreesSeed = Math.toDegrees(Constants.FRONT_LEFT_MODULE_STEER_OFFSET);

    public SteerMotorCanCoderSubsystem() {
        canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
        canCoderConfig.MagnetSensor.MagnetOffset = CtreUtils.convertFromRadiansToNormalizedDecmil(encoderOffsetDegreesSeed);
        canCoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;
        CtreUtils.checkCtreError(encoder.getConfigurator().apply(canCoderConfig, 250), "Failed to configure CANCoder");
//        encoder.clearStickyFaults();
        SmartDashboard.putNumber("SteerEncoder_ModuleOffsetUpdate", this.encoderOffsetDegreesSeed);
    }
 
    @Override
    public void periodic() {
        ++counter;
        if(counter % 10 == 0) {
            this.publishStaticistics();
        }
        if(counter % 100 == 0) {
            this.updateSeedValueFromDashboard();
        }
    }

    @Override
    public void simulationPeriodic() {
        // This method will be called once per scheduler run during simulation

    }

    private void publishStaticistics() {
        SmartDashboard.putNumber("SteerEncoder_ModuleStaticOffset", Constants.FRONT_LEFT_MODULE_STEER_OFFSET);
        SmartDashboard.putNumber("SteerEncoder_ModuleOffset", this.encoderOffsetDegreesSeed);
        SmartDashboard.putString("SteerEncoder_Name", encoder.getAppliedControl().getName());
        SmartDashboard.putNumber("SteerEncoder_Voltage", encoder.getSupplyVoltage().getValueAsDouble());
        SmartDashboard.putNumber("SteerEncoder_AbsolutePosition", encoder.getAbsolutePosition().getValueAsDouble());
        SmartDashboard.putNumber("SteerEncoder_Position", encoder.getPosition().getValueAsDouble());
        SmartDashboard.putNumber("SteerEncoder_PositionSinceBoot", encoder.getPositionSinceBoot().getValueAsDouble());
        SmartDashboard.putNumber("SteerEncoder_UnfilteredVelocity", encoder.getUnfilteredVelocity().getValueAsDouble());
        SmartDashboard.putNumber("SteerEncoder_Velocity", encoder.getVelocity().getValueAsDouble());
    }
 
    private void updateSeedValueFromDashboard()
    {
        double dashboardValue = SmartDashboard.getNumber("SteerEncoder_ModuleOffsetUpdate", this.encoderOffsetDegreesSeed);
        if(dashboardValue != this.encoderOffsetDegreesSeed) {
            System.out.println("updated offset degrees from " + this.encoderOffsetDegreesSeed + " to " + dashboardValue);
            this.encoderOffsetDegreesSeed = dashboardValue;
            double updateValue = CtreUtils.convertFromRadiansToNormalizedDecmil(Math.toRadians(this.encoderOffsetDegreesSeed));
            System.out.println("new MagnetSensor.MagnetOffset == " + updateValue);
            this.canCoderConfig.MagnetSensor.MagnetOffset = updateValue;
            SmartDashboard.putNumber("SteerEncoder_LastUpdateValue", updateValue);
            CtreUtils.checkCtreError(this.encoder.getConfigurator().apply(canCoderConfig, 250), "Failed to configure CANCoder");

            this.encoderOffsetDegreesSeed = 0.0;
            SmartDashboard.putNumber("SteerEncoder_ModuleOffsetUpdate", this.encoderOffsetDegreesSeed);
       }
    }
}