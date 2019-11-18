/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import frc.robot.Auto.DrivetrainOdometry;
import frc.robot.Auto.LiveDashboard;
import frc.robot.Misc.Constants;
import frc.robot.Misc.OI;

public class Robot extends TimedRobot {

  public static Drivetrain drivetrain;
  public static LiveDashboard falcondashboard;
  public static OI oi;

  public static DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.trackwidth);

  public static DrivetrainOdometry odometry;
  private static Notifier odometryNotifier;

  TrajectoryConfig config = new TrajectoryConfig(Constants.kTopSpeedMPS-1, 3);
  
  Trajectory right_side_auto;
  Pose2d[] right_side_auto_waypoints = {new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                            new Pose2d(3, 0, Rotation2d.fromDegrees(0))};
                      
  
  

  @Override
  public void robotInit() {
    drivetrain = Drivetrain.getInstance();
    oi = OI.getInstance();

    falcondashboard = new LiveDashboard();

    kinematics = new DifferentialDriveKinematics(Constants.trackwidth);
    odometryNotifier = new Notifier(odometry);
    odometryNotifier.startPeriodic(0.02);
    
  }

  @Override
  public void autonomousInit() {
    
  }

  @Override
  public void autonomousPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    Scheduler.getInstance().run();
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
