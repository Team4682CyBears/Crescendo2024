package frc.robot.common;

import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.playingwithfusion.TimeOfFlight;
import com.playingwithfusion.TimeOfFlight.RangingMode;
import frc.robot.control.HardwareConstants;

/**
 * A class intending to help keep CAN usage tidy
 */
public class CanUtils {

    /**
     * A method to update the TalonFX motor frequency to team-wide approved setting
     * @param motor - the TalonFX motor (usually not a follower motor)
     */
    public static void UpdateCtreFrequency(TalonFX motor) {
        double currentFrequency = motor.getPosition().getAppliedUpdateFrequency();
        if(currentFrequency < HardwareConstants.ctreMotorStatusFramePeriodFrequencyHertz) {
            motor.getPosition().setUpdateFrequency(HardwareConstants.ctreMotorStatusFramePeriodFrequencyHertz);
            System.out.println(
                "Updating frequency for TalonFX # " +
                motor.getDeviceID() +
                " from " + currentFrequency +
                " to " + HardwareConstants.ctreMotorStatusFramePeriodFrequencyHertz);
        }
        else {
            System.out.println(
                "No frequency change for TalonFX # " +
                motor.getDeviceID() +
                " existing frequency == " + currentFrequency);
        }
    }

    /**
     * A method to update the TalonFX motor frequency to team-wide approved setting
     * @param motor - the TalonFX motor (usually not a follower motor)
     */
    public static void UpdateCtreFrequency(TalonSRX motor) {
        int currentPeriodMilliseconds = motor.getStatusFramePeriod(StatusFrameEnhanced.Status_1_General);
        if(currentPeriodMilliseconds < HardwareConstants.ctreMotorStatusFramePeriodMilliseconds) {
            motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, HardwareConstants.ctreMotorStatusFramePeriodMilliseconds);
            System.out.println(
                "Updating period (in milliseconds) for TalonSRX # " +
                motor.getDeviceID() +
                " from " + currentPeriodMilliseconds +
                " to " + HardwareConstants.ctreMotorStatusFramePeriodMilliseconds);
        }
        else {
            System.out.println(
                "No period change for TalonSRX # " +
                motor.getDeviceID() +
                " existing period == " + currentPeriodMilliseconds);
        }
    }

    /**
     * A method to update the CANcoder sensor frequency to team-wide approved setting
     * @param canCoder - the CANcoder sensor
     */
    public static void UpdateCtreFrequency(CANcoder canCoder) {
        double currentFrequency = canCoder.getPosition().getAppliedUpdateFrequency();
        if(currentFrequency < HardwareConstants.ctreSensorStatusFramePeriodFrequencyHertz) {
            canCoder.getPosition().setUpdateFrequency(HardwareConstants.ctreSensorStatusFramePeriodFrequencyHertz);
            System.out.println(
                "Updating frequency for CANCoder # " +
                canCoder.getDeviceID() +
                " from " + currentFrequency +
                " to " + HardwareConstants.ctreMotorStatusFramePeriodFrequencyHertz);
        }
        else {
            System.out.println(
                "No frequency change for CANCoder # " +
                canCoder.getDeviceID() +
                " existing frequency == " + currentFrequency);
        }
    }

    /**
     * A method to update the TimeOfFlight sample rate to team-wide approved setting
     * @param sensor - the TimeOfFlight
     */
    public static void UpdateTofFrequency(TimeOfFlight sensor) {
        RangingMode currentMode = sensor.getRangingMode(); 
        double currentPeriodMilliseconds = sensor.getSampleTime();
        if(currentPeriodMilliseconds < HardwareConstants.playingWithFusionSensorPeriodMilliseconds) {
            sensor.setRangingMode(currentMode, HardwareConstants.playingWithFusionSensorPeriodMilliseconds);
            System.out.println(
                "Updating period for TimeOfFlight serial # " +
                sensor.getSerialNumber() +
                " from " + currentPeriodMilliseconds +
                " to " + HardwareConstants.playingWithFusionSensorPeriodMilliseconds);
        }
        else {
            System.out.println(
                "No period change for TimeOfFlight serial # " +
                sensor.getSerialNumber() +
                " existing period == " + currentPeriodMilliseconds);
        }
    }
}
