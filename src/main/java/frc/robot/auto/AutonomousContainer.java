package frc.robot.auto;

import java.util.List;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import frc.robot.misc.Constants;
import frc.robot.subsystems.drive.Drivetrain;

public class AutonomousContainer {
    private static TrajectoryConfig config = new TrajectoryConfig(Constants.kTopSpeedMPS, 20) // These numbers literally do not matter
            .addConstraint(
                    new DifferentialDriveVoltageConstraint(Drivetrain.motorFeedForward, Drivetrain.kinematics, 9));

    public static Trajectory right_side_auto;
    private static Pose2d[] points = { new Pose2d(0, 0, Rotation2d.fromDegrees(0)), 
                                       new Pose2d(3.929, 2.084, Rotation2d.fromDegrees(90)) };

    private AutonomousContainer() {
        right_side_auto = TrajectoryGenerator.generateTrajectory(List.of(points), config);
    }

    private static AutonomousContainer instance;
    public static AutonomousContainer getInstance() {
        if (instance == null) instance = new AutonomousContainer();
        return instance;
    }

    public Trajectory getAutonomousTrajectory() {
        return right_side_auto;
    }
}