package frc.robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Misc.Constants;
import frc.robot.Misc.OI;

public class Drive extends Command {

    private double left, right;
    private State state;
    private double last_left = 0, last_right = 0;

    public Drive(State state) {
        this.state = state;
        requires(Robot.drivetrain);
    }

    // Every time execute() is called by the Scheduler, the current XBox controller
    // joystick values are used to calculate motor speeds
    protected void execute() {

        // Retrieving the deadbanded throttle and turn values (the controller joystick values)
        double throttle = OI.getThrottleValue() * Constants.kDriveSens;
        double turn = OI.getTurnValue();

        SmartDashboard.putNumber("throttle", throttle);
        SmartDashboard.putNumber("turn", turn);

        switch(state){
            case OpenLoop:
                // Differential drive as long as throttle is greater than zero (deadbanded). 
                if (throttle != 0) {
                    left = throttle + throttle * turn * Constants.kTurnSens;
                    right = throttle - throttle * turn * Constants.kTurnSens;

                // Turns in place when there is no throttle input
                } else {
                    left = turn * Constants.kTurnInPlaceSens;
                    right = -turn * Constants.kTurnInPlaceSens;
                }

                SmartDashboard.putNumber("left", left);
                SmartDashboard.putNumber("right", right);

                Drivetrain.setOpenloop(left, right);
                break;

            case CheesyDrive:
                // Cheesydrive as long as throttle is greater than zero (deadbanded)
                if (throttle != 0) {
                    double nu = throttle*Constants.kTopSpeedFPS;
                    double omega = nu*Math.toRadians(turn*Constants.kCurvatureScale);

                    SmartDashboard.putNumber("omega", omega);

                    left = nu + (Constants.wheelbase / 2.0) * omega;
                    right = nu - (Constants.wheelbase / 2.0) * omega;

                    /*
                    V_app = kS + kV * velocity + kA * acceleration;
                    kS is multiplied by signum(velocity), which returns 0 when desired velocity is 0 
                    */
                    double leftFf = (Constants.kS * Math.signum(left) + Constants.kV * left + Constants.kA * (left - last_left)/0.02)/12;
                    double rightFf = (Constants.kS * Math.signum(right) + Constants.kV * right + Constants.kA * (right - last_right)/0.02)/12;

                    last_left = left;
                    last_right = right;

                    // Converting velocities to Talon native velocity units
                    left = FPStoTicksPerDecisecond(left);
                    right = FPStoTicksPerDecisecond(right);

                    Drivetrain.setClosedloop(left, leftFf, right, rightFf);

                // Turns in place when there is no throttle input
                } else {
                    left = turn * Constants.kTurnInPlaceSens;
                    right = -turn * Constants.kTurnInPlaceSens;

                    Drivetrain.setOpenloop(left, right);
                }
                
                
                break;
        }
        
    }

    // Because this Command is default, it never needs to end -- it will simply be interrupted whenever another Command requires the drivetrain
    protected boolean isFinished() {
        return false;
    }

    // When this command ends, it stops the drivetrain to guarantee safety
    protected void end() {
        Drivetrain.setOpenloop(0, 0);
    }

    protected static enum State {
        OpenLoop, CheesyDrive
    }

    /**
     * Converts feet/second to ticks/100ms
     * @param fps feet/second input
     * @return equivalent in ticks/100ms 
     */
    private double FPStoTicksPerDecisecond(double fps){
        return fps * 12 / (4 * Math.PI) * 1024 / 10;
    }

}