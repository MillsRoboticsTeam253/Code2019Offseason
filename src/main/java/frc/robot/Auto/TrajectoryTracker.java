package frc.robot.Auto;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Robot;
import jaci.pathfinder.Trajectory;

public class TrajectoryTracker extends Command {

    private Notifier notifier;
    private double dt;
    private Trajectory traj;

    public TrajectoryTracker(double dt, Trajectory traj){
        this.notifier = new Notifier(this::update);
        this.dt = dt;
        requires(Robot.drivetrain);
    }

    protected void initialize(){
        notifier.startPeriodic(dt);
    }

    private void update(){

    }

    @Override
    protected boolean isFinished() {
        return false;
    }


}