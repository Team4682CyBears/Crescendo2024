package frc.robot.common;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import frc.robot.control.InstalledHardware;

/**
 * A collection of methods for vector computations
 */
public class VectorUtils{

    /**
     * 3d vector cross product
     * @param a Translation3d - a vector
     * @param b Translation3d - a vector
     * @return a x b
     */
    public static Translation3d cross(Translation3d a, Translation3d b){
        return new Translation3d(
            (-1 * b.getY() * a.getZ()) + (b.getZ() * a.getY()),
            (b.getX() * a.getZ()) - (b.getZ() * a.getX()),
            (-1 * b.getX() * a.getY()) + (b.getY() * a.getX()));
    }

    /**
     * Rotates a 3d vector by a quaternion
     * @param vec - 3d vector
     * @param q - quaternion
     * @return rotated 3d vector
     */
    public static Translation3d rotateByQuaternion(Translation3d vec, Quaternion q){
        // create a quaternion from the vector
        final Quaternion p = new Quaternion(0.0d, vec.getX(), vec.getY(), vec.getZ());
        final Quaternion r = q.times(p).times(q.inverse());
        return new Translation3d(r.getX(), r.getY(), r.getZ());
    }

    /**
     * Return a vector represting the angle of steepest ascent given pitch,roll, yaw
     * @param robotPose EulerAngle (pitch, roll, yaw)
     * @return Translation2d - the x,y vector specifying the direction of ascent
     */
    public static Translation2d getAngleOfSteepestAscent(EulerAngle robotPose)
    {
        // apply deadband to pitch/roll so that angles within tolerence are handled like 0
        double tolDegrees = 2.0;
        double roll = (Math.abs(robotPose.getRoll()) >= tolDegrees) ? robotPose.getRoll() : 0.0;
        double pitch = (Math.abs(robotPose.getPitch()) >= tolDegrees) ? robotPose.getPitch() : 0.0;

        if (InstalledHardware.navx1Installed){
            // Move X proportional to the sin of the roll (rotation about y)
            // Move Y proportional to the sin or the pitch (rotation about x)
            return new Translation2d(
                Math.sin(Math.toRadians(roll)),
                Math.sin(Math.toRadians(pitch)));
        }
        else { //navx2 istalled
            // Move Y proportional to the sin of the roll (rotation about x)
            // Move X proportional to the sin or the pitch (rotation about y)
            return new Translation2d(
                Math.sin(Math.toRadians(pitch)),
                -1 * Math.sin(Math.toRadians(roll)));
        }
    }

    /**
     * Translates the x,y portion of the pose. Leaves the rotation unchanged.
     * Use instead of Pose2d.add, which does strange things when the rotation is non-zero.
     * @param pose
     * @param translation
     * @return new pose
     */
    public static Pose2d translatePose(Pose2d pose, Translation2d translation){
        return new Pose2d(pose.getTranslation().plus(translation), pose.getRotation());
    }
}
