package frc.robot.common;

import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation3d;

/**
 * A type to hold Euler angles (Pitch, Yaw, Roll)
 * pitch is rotation around x axis
 * roll is rotation around y axis
 * yaw is rotation around z axis
 */
public class EulerAngle {
    private double pitch;
    private double roll;
    private double yaw;

    /**
     * Constructs a set of Euler Angles from pitch, roll, yaw
     * @param pitch
     * @param roll
     * @param yaw
     */
    public EulerAngle(double pitch, double roll, double yaw)
    {
        this.pitch = pitch;
        this.yaw = yaw;
        this.roll = roll;
    }

    /**
     * Sets pitch
     * @param pitch
     */
    public void setPitch(double pitch)
    {
        this.pitch = pitch;
    }

    /**
     * gets pitch
     * @return pitch
     */
    public double getPitch()
    {
        return pitch;
    }

    /**
     * sets roll
     * @param roll
     */
    public void setRoll(double roll)
    {
        this.roll = roll;
    }

    /**
     * gets roll
     * @return roll
     */
    public double getRoll()
    {
        return roll;
    }

    /**
     * sets yaw
     * @param yaw
     */
    public void setYaw(double yaw)
    {
        this.yaw = yaw;
    }

    /**
     * gets yaw
     * @return yaw
     */
    public double getYaw()
    {
        return yaw;
    }

    /**
     * converts euler angles to quaternion
     * @return quaternion
     */
    public Quaternion getQuaternion()
    {
        return new Rotation3d(pitch, yaw, roll).getQuaternion();
    }

    @Override
    public String toString() {
        return "EulerAngle(Pitch: " + pitch + ", Roll: " + roll + ", Yaw: " + yaw + ")";
      }
    
}
