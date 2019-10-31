package frc.robot.Auto;

import java.io.File;
import java.io.IOException;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.first.wpilibj.command.Command;
import frc.robot.Drivetrain;
import frc.robot.Robot;
import frc.robot.Misc.Constants;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Trajectory.Segment;

import static java.lang.Math.sin;
import static java.lang.Math.cos;
import static java.lang.Math.sqrt;

public class TrajectoryTracker extends Command {

    private Trajectory trajec;

    private double dt;
    private double beta;
    private double zeta;

    private int index;

    /**
     * Uses a time-varying non-linear reference controller to steer the robot back
     * onto the trajectory. 
     * From https://www.dis.uniroma1.it/~labrob/pub/papers/Ramsete01.pdf eq. 5.12 
     * Further information https://file.tavsys.net/control/state-space-guide.pdf Theorum 8.52
     * 
     * @param pathName Name of the trajectory csv to be run, automatically looks in
     *                 the /home/lvuser/profiles/ directory, does not need to
     *                 include .csv file ext
     * @param beta     Constant for correction, increase for stronger convergence
     * @param zeta     Constant for dampening, increase for stronger dampening
     */
    public TrajectoryTracker(String pathName, double beta, double zeta) {
        this.beta = beta;
        this.zeta = zeta;

        // This command takes control of the drivetrain when run
        requires(Robot.drivetrain);

        // Attempts to load paths from filesystem
        try {
            this.trajec = Pathfinder.readFromCSV(new File("/home/lvuser/profiles/" + pathName + ".csv"));
        } catch (IOException e) {
            System.out.println("Path loading failed");
        }

        this.dt = trajec.get(0).dt;

    }

    // Alternate constructor that takes in a Trajectory object instead of a path name and dt
    public TrajectoryTracker(Trajectory trajec, double beta, double zeta) {
        this.trajec = trajec;
        this.beta = beta;
        this.zeta = zeta;
        this.dt = trajec.get(0).dt;

        // This command takes control of the drivetrain when run
        requires(Robot.drivetrain);

    }

    // When the Command is started, the trajectory segment index is reset to zero
    protected void initialize() {
        index = 0;
    }

    // Iterates through the Trajectory at intervals of dt, using Ramsete to
    // calculate the proper drivetrain velocities to steer the robot to the path
    protected void execute() {
        // Gets reference pose from Trajectory Segments
        Segment reference = trajec.get(index);
        // Retrieving current pose from EncoderOdom class
        Pose2D actual = EncoderOdom.getPose();

        /* FalconDashboard visualization */
        Robot.falcondashboard.putPath(reference.x, reference.y, reference.heading);

        /* Ramsete formulas */
        double vd = reference.velocity;
        // Deriving reference angular velocity using calculus uwu
        double wd = index == 0 ? reference.velocity * (reference.heading) / dt
                : reference.velocity * (reference.heading - trajec.get(index - 1).heading) / dt;

        // Calculating gain
        double k1 = 2 * zeta * sqrt(wd*wd + beta*vd*vd);
        
        // Calculating linear velocity required to get to Trajectory
        double v = vd * cos(reference.heading - actual.heading) + k1
                * ((reference.x - actual.x) * cos(actual.heading) + (reference.y - actual.y) * sin(actual.heading));

        // Calculating angular velocity required to get to Trajectory
        double w = wd + beta * vd * sinc(boundRadian(reference.heading - actual.heading)
                * ((reference.y - actual.y) * cos(actual.heading) - (reference.x - actual.x) * sin(actual.heading))
                + k1 * (boundRadian(reference.heading - actual.heading)));

        // Moves index to next segment if there are segments remaining
        if(index < trajec.segments.length-1) index++;

        /**
         * Ramsete's outputs are linear velocity and angular velocity, which make sense
         * because Ramsete is a unicycle controller (one wheel, one velocity). We
         * convert these to left and right wheel velocities for use on a differentially
         * driven robot using inverse kinematics:
         */
        double left = v - (Constants.wheelbase / 2.0) * w;
        double right = v + (Constants.wheelbase / 2.0) * w;

        // Converting velocities to Talon native velocity units
        left = FPStoTicksPerDecisecond(left);
        right = FPStoTicksPerDecisecond(right);

        // Setting drivetrain to speeds in closed loop
        Drivetrain.set(ControlMode.Velocity, left, right);

    }

    // Returns true if the index has reached the last segment
    @Override
    protected boolean isFinished() {
        return index == trajec.segments.length - 1;
    }

    // When isFinished() returns true, the command will stop running and the drivetrain will be set to stop
    protected void end() {
        Drivetrain.set(ControlMode.Velocity, 0.0, 0.0);
    }

    /**
     * Implementation of the cardinal sine function, sin(x)/x. Because this function is undefined at zero, we switch to the approximation of this function 1-(x^2)/6 when we approach zero.
     * 
     * @param theta Input angle measure
     * @return cardinal sine of @param theta
     */
    private double sinc(double theta){
        return epsilonEquals(theta, 0.0) ? 1.0 - 1.0 / 6.0 * theta * theta : sin(theta)/theta;
    }

    /**
     * Because floating point math can sometimes lead to imprecision, we determine whether two numbers are equal by determining if their difference is less than a constant (1E-9).
     * 
     * @param lhs first value
     * @param rhs second value
     * @return if @param lhs and @param rhs can be assumed equal
     */
    private boolean epsilonEquals(double lhs, double rhs) {
        return Math.abs(lhs-rhs) < 1E-9;
    }

    /**
     * Converts feet/second to ticks/100ms
     * @param fps feet/second input
     * @return equivalent in ticks/100ms 
     */
    private double FPStoTicksPerDecisecond(double fps){
        return fps * 12 / (4 * Math.PI) * 1024 / 10;
    }

    /**
     * Bounds a radian measure to -pi to pi
     * @param rad the radian input
     * @return equivalent angle bounded to the range
     */
    private double boundRadian(double rad){
        while(rad > Math.PI) rad -= 2*Math.PI;
        while(rad < -Math.PI) rad += 2*Math.PI;
        return rad;
    }

}