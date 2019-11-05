package frc.robot.Auto;


public class Pose2D {
    public double x, y, heading;

    /**
     * Container class used with {@link TrajectoryTracker} 
     * 
     * @param x The x component of pose
     * @param y The y component of pose
     * @param theta The heading component of pose
     */
    public Pose2D(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

}