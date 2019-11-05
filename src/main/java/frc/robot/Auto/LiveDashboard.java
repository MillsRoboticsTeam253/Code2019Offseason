package frc.robot.Auto;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

// Interfaces with FalconDashboard 
// https://github.com/5190GreenHopeRobotics/FalconDashboard

public class LiveDashboard {
    private NetworkTableEntry robotX;
    private NetworkTableEntry robotY;
    private NetworkTableEntry robotHeading;

    private NetworkTableEntry followingPath;
    private NetworkTableEntry pathX;
    private NetworkTableEntry pathY;
    private NetworkTableEntry pathHeading;

    public void setup(){
        NetworkTable table = NetworkTableInstance.getDefault().getTable("Live_Dashboard");
        robotX = table.getEntry("robotX");
        robotY = table.getEntry("robotY");
        robotHeading = table.getEntry("robotHeading");

        followingPath = table.getEntry("isFollowingPath");
        pathX = table.getEntry("pathX");
        pathY = table.getEntry("pathY");
        pathHeading = table.getEntry("pathHeading");

        endPath();
        
    }

    public void putOdom(double robotX, double robotY, double robotHeading) {
        this.robotX.setDouble(robotX);
        this.robotY.setDouble(robotY);
        this.robotHeading.setDouble(robotHeading);
    }

    public void putOdom(Pose2D pose){
        putOdom(pose.x, pose.y, pose.heading);
    }

    public void putPath(double pathX, double pathY, double pathHeading){
        this.pathX.setDouble(pathX);
        this.pathY.setDouble(pathY);
        this.pathHeading.setDouble(pathHeading);
        this.followingPath.setBoolean(true);
    }

    public void endPath(){
        this.followingPath.setBoolean(false);
    }




}