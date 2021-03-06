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
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.auto.AutonomousContainer;
import frc.robot.auto.LiveDashboard;
import frc.robot.auto.TrajectoryTracker;
import frc.robot.misc.OI;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.drive.Drivetrain;

public class Robot extends TimedRobot {

    public static Drivetrain drivetrain;
    public static LiveDashboard falcondashboard;
    public static OI oi;
    public static AHRS navX;

    private static AutonomousContainer auto;

    

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
    }

    @Override
    public void robotInit() {
        navX = new AHRS(Port.kMXP); // Initialized first because Drivetrain requires the gyro for initialiation

        drivetrain = Drivetrain.getInstance();
        drivetrain.setDefaultCommand(new Drive(Drive.State.CheesyDrive));
        oi = OI.getInstance();

        falcondashboard = new LiveDashboard();
        falcondashboard.setup();

        auto = AutonomousContainer.getInstance();
    }

    @Override
    public void autonomousInit() {
        // Begins odometry at the beginning of autonomous period
        CommandScheduler.getInstance().registerSubsystem(drivetrain);
        CommandScheduler.getInstance().schedule(new TrajectoryTracker(AutonomousContainer.getInstance().getAutonomousTrajectory()));
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
