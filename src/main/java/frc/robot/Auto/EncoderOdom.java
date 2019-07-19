package frc.robot.Auto;

import frc.robot.Drivetrain;
import frc.robot.Misc.OI;

public class EncoderOdom implements Runnable {
    private Pose2D pose;
    private double lastLeft;
    private double lastRight;

    public EncoderOdom() {
        this.pose = new Pose2D(0, 0, 0);

    }

    public void clear() {
        lastLeft = 0.0;
        lastRight = 0.0;
        pose = new Pose2D(0, 0, 0);
    }

    public void setup() {
        lastLeft = Drivetrain.getLeftFeet();
        lastRight = Drivetrain.getRightFeet();
    }

    public void run() {
        double dl = Drivetrain.getLeftFeet() - lastLeft; // change in left travel
        double dr = Drivetrain.getRightFeet() - lastRight; // change in right travel

        lastLeft = Drivetrain.getLeftFeet();
        lastRight = Drivetrain.getRightFeet();

        double vel = (dl + dr) / 2.0; // averaging traveled distance to find velocity

        // Changing the pose based on traveled distances
        pose.setX(pose.getX() + vel * Math.cos(OI.getGyroRadians()));
        pose.setY(pose.getY() + vel * Math.sin(OI.getGyroRadians()));
        pose.setTheta(OI.getGyroRadians());

    }
}