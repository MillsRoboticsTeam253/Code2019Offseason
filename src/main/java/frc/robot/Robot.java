/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Scheduler;
import frc.robot.Auto.LiveDashboard;
import frc.robot.Misc.OI;

public class Robot extends TimedRobot {

  public static Drivetrain drivetrain;
  public static LiveDashboard falcondashboard;
  public static OI oi;

  @Override
  public void robotInit() {
    drivetrain = Drivetrain.getInstance();
    oi = OI.getInstance();

    falcondashboard = new LiveDashboard();
    
  }

  @Override
  public void autonomousInit() {
    // Figure out starting position and use that to set the offset
    drivetrain.odometry.clear();
    drivetrain.resetEncoders();
    falcondashboard.setup();
    
  }

  @Override
  public void autonomousPeriodic() {
    // Updating robot pose using odometry
    drivetrain.odometry.run(); 

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
