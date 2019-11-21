/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.constraint.DifferentialDriveVoltageConstraint;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Auto.LiveDashboard;
import frc.robot.Auto.TrajectoryTracker;
import frc.robot.Drive.State;
import frc.robot.Misc.Constants;
import frc.robot.Misc.OI;

public class Robot extends TimedRobot {

  public static Drivetrain drivetrain;
  public static LiveDashboard falcondashboard;
  public static OI oi;
  public static AHRS navX = new AHRS(edu.wpi.first.wpilibj.I2C.Port.kMXP);

  TrajectoryConfig config = new TrajectoryConfig(Constants.kTopSpeedMPS, 20) // These numbers literally do not matter 
    .addConstraint(new DifferentialDriveVoltageConstraint(Drivetrain.motorFeedForward, 
                                                          Drivetrain.kinematics, 
                                                          11));
  
  Trajectory right_side_auto;
  Pose2d[] points = {new Pose2d(0, 0, Rotation2d.fromDegrees(0)),
                            new Pose2d(3, 0, Rotation2d.fromDegrees(0))};
                      
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }
  

  @Override
  public void robotInit() {
    // Initialized first because Drivetrain requires the gyro for initialiation

    drivetrain = Drivetrain.getInstance();
    CommandScheduler.getInstance().registerSubsystem(drivetrain);
    oi = OI.getInstance();

    falcondashboard = new LiveDashboard();
    right_side_auto = TrajectoryGenerator.generateTrajectory(List.of(points), config);

    drivetrain.setDefaultCommand(new Drive(State.OpenLoop));
    
  }

  @Override
  public void autonomousInit() {
    // Begins odometry at the beginning of autonomous period
    CommandScheduler.getInstance().registerSubsystem(drivetrain);
    CommandScheduler.getInstance().schedule(new TrajectoryTracker(right_side_auto));
  }

  @Override
  public void autonomousPeriodic() {
    
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

}
