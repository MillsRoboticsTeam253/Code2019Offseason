package frc.robot.Auto;

import edu.wpi.first.wpilibj.Notifier;
import frc.robot.Drivetrain;
import frc.robot.Misc.OI;

public class EncoderOdom implements Runnable {
    private static Pose2D pose;
    private double lastLeft;
    private double lastRight;

    private Notifier notifier;

    /**
     * Runnable class used to update the robot's pose asynchronously from the main robot loop
     */
    public EncoderOdom() {
        pose = new Pose2D(0, 0, 0);
        notifier = new Notifier(this);
    }

    // When this method is called, the notifier begins to update the pose
    public void start(double dt){
        notifier.startPeriodic(dt);
    }

    // When this method is called, the notifier stops updating pose
    public void stop(){
        notifier.stop();
    }

    // Resets pose to zero
    public void clear() {
        lastLeft = 0.0;
        lastRight = 0.0;
        pose = new Pose2D(0, 0, 0);
    }

    // Sets initial distance traveled offset
    public void setup() {
        lastLeft = Drivetrain.getLeftFeet();
        lastRight = Drivetrain.getRightFeet();
    }

    public void run() {
        // Calculating the change in each side of the drivetrain by comparing the current distance to the last distance
        double dl = Drivetrain.getLeftFeet() - lastLeft; 
        double dr = Drivetrain.getRightFeet() - lastRight; 

        // Setting the new last distances
        lastLeft = Drivetrain.getLeftFeet();
        lastRight = Drivetrain.getRightFeet();

        // Calculating velocity by averaging the distance traveled on each side of the drivetrain
        double vel = (dl + dr) / 2.0;

        // Updating the pose based on traveled distances
        pose.x = pose.x + vel * Math.cos(OI.getGyroRadians());
        pose.y = pose.y + vel * Math.sin(OI.getGyroRadians());
        pose.heading = OI.getGyroRadians();

    }

    public static Pose2D getPose() {
        return pose;
    }
}