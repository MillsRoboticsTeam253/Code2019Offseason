package frc.robot.Auto;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.util.Units;
import frc.robot.Drivetrain;
import frc.robot.Robot;
import frc.robot.Misc.Constants;

public class DrivetrainOdometry extends DifferentialDriveOdometry implements Runnable {

    private static DifferentialDriveKinematics kinematics = Robot.kinematics;
    private AHRS navX;

    public DrivetrainOdometry(AHRS navX, Pose2d initialPoseMeters) {
        super(kinematics, new Rotation2d(), initialPoseMeters);
        this.navX = navX;
        super.resetPosition(initialPoseMeters, new Rotation2d(Units.degreesToRadians(navX.getAngle())));
    }

    public DrivetrainOdometry(AHRS navX) {
        super(kinematics, new Rotation2d(), new Pose2d());
        this.navX = navX;
        super.resetPosition(new Pose2d(), new Rotation2d(Units.degreesToRadians(navX.getAngle())));
    }

    @Override
    public void run() {
        double leftVelocity = TicksPerDecisecondtoMPS(Drivetrain.getLeftEncVelocity());
        double rightVelocity = TicksPerDecisecondtoMPS(Drivetrain.getRightEncVelocity());

        DifferentialDriveWheelSpeeds wheelSpeeds = new DifferentialDriveWheelSpeeds(leftVelocity, rightVelocity);
        Rotation2d gyroAngle = new Rotation2d(Units.degreesToRadians(navX.getAngle()));

        super.updateWithTime(Timer.getFPGATimestamp(), gyroAngle, wheelSpeeds);
        Robot.falcondashboard.putOdom(super.getPoseMeters(), gyroAngle);
    }

    private static double TicksPerDecisecondtoMPS (double val) {
        return val * 62.8318530718 * Constants.wheelRadius / Constants.ticksPerRotation;
    }

}