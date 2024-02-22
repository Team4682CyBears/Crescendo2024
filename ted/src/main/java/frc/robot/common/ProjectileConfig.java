// ************************************************************
// Bishop Blanchet Robotics
// Home of the Cybears
// FRC - Crescendo - 2024
// File: ProjectileConfig.java
// Intent: Forms a type to hold an angle and speed
// ************************************************************

// ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ ʕ •ᴥ•ʔ ʕ•ᴥ•  ʔ ʕ  •ᴥ•ʔ ʕ •`ᴥ´•ʔ ʕ° •° ʔ 

package frc.robot.common;

/**
 * Forms a type to hold an analge and speed
 */
public class ProjectileConfig {
    private double angle;
    private double speed;

    /**
     * Constructs a ProjectileConfig from angle, speed
     * 
     * @param angle
     * @param speed
     */
    public ProjectileConfig(double angle, double speed) {
        this.angle = angle;
        this.speed = speed;
    }

    /**
     * A method to get the angle
     * 
     * @return angle
     */
    public double getAngle() {
        return angle;
    }

    /**
     * A method to get the speed
     * 
     * @return speed
     */
    public double getSpeed() {
        return speed;
    }

    /**
     * A method to set the angle
     * 
     * @param angle
     */
    public void setAngle(double angle) {
        this.angle = angle;
    }

    /**
     * A method to set the speed
     * 
     * @param speed
     */
    public void setSpeed(double speed) {
        this.speed = speed;
    }

    /**
     * A method to return a string with the angle and speed
     * 
     * @return string
     */
    @Override
    public String toString() {
        return "ProjectileConfig Angle: " + angle + ", Speed: " + speed;
    }

}
