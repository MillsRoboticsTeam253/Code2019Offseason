package frc.robot.Auto;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import frc.robot.Drivetrain;
import frc.robot.Robot;

public class TrajectoryTracker extends RamseteCommand {

    static DifferentialDriveKinematics kinematics = Robot.kinematics;
    static RamseteController follower = new RamseteController(2, 0.7);
    private double startTime;
    private Trajectory trajectory;

    public TrajectoryTracker(Trajectory trajectory) {
        super(trajectory, 
              () -> Robot.odometry.getPoseMeters(), 
              follower, 
              kinematics, 
              (left, right) -> Drivetrain.setOpenLoop(left, right));
        this.trajectory = trajectory;
        
    }

    @Override 
    public void initialize() {
        startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute() {
        super.execute();
        double currentTime = Timer.getFPGATimestamp();

        Trajectory.State currentState = trajectory.sample(currentTime-startTime);
        Pose2d currentPose = currentState.poseMeters;
        Rotation2d currentRotation = currentPose.getRotation();
        Robot.falcondashboard.putPath(currentPose, currentRotation);
    }

    
}