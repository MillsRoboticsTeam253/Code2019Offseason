/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import frc.robot.Misc.OI;

public class Robot extends TimedRobot {

  public static Drivetrain drivetrain;
  public static OI oi;

  @Override
  public void robotInit() {
    drivetrain = Drivetrain.getInstance();
    oi = OI.getInstance();
    
  }

  // Begins odometry when autonomous mode is initialized, updating at 100Hz
  @Override
  public void autonomousInit() {
    drivetrain.odometry.start(0.01);
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
