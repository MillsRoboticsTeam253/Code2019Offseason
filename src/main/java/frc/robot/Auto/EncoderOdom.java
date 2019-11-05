package frc.robot.Auto;

import frc.robot.Drivetrain;
import frc.robot.Robot;
import frc.robot.Misc.OI;

public class EncoderOdom implements Runnable {
    private static Pose2D pose;
    private double lastLeft;
    private double lastRight;

    /**
     * Runnable class used to update the robot's pose asynchronously from the main robot loop
     */
    public EncoderOdom() {
        pose = new Pose2D(0, 0, 0);
    }

    // Resets pose to zero
    public void clear() {
        lastLeft = 0.0;
        lastRight = 0.0;
        pose = new Pose2D(0, 0, 0);
    }

    // Sets initial position offset
    public void setup(double offsetX, double offsetY) {
        pose.x = offsetX;
        pose.y = offsetY;
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

        Robot.falcondashboard.putOdom(pose.x, pose.y, pose.heading);

    }

    public static Pose2D getPose() {
        return pose;
    }
}