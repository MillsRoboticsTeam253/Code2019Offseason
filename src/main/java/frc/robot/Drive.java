package frc.robot;

import java.util.Set;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Drivetrain.WheelState;
import frc.robot.Misc.Constants;
import frc.robot.Misc.OI;

public class Drive implements Command {

    private double left, right;
    private State state;

    public Drive(State state) {
        this.state = state;
        
    }

    Subsystem[] requirements = {Robot.drivetrain};

    // Every time execute() is called by the Scheduler, the current XBox controller
    // joystick values are used to calculate motor speeds
    public void execute() {

        // Retrieving the deadbanded throttle and turn values (the controller joystick
        // values)
        double throttle = OI.getThrottleValue();
        double turn = OI.getTurnValue();

        SmartDashboard.putNumber("throttle", throttle);
        SmartDashboard.putNumber("turn", turn);

        switch (state) {
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

                Drivetrain.setOpenLoop(left, right);
                break;

            case CheesyDrive:

                WheelState wheelSpeeds;
                if (throttle != 0) {
                    wheelSpeeds = Drivetrain.DifferentialDrive.curvatureDrive(throttle, turn, false);
                } else {
                    wheelSpeeds = Drivetrain.DifferentialDrive.curvatureDrive(throttle, turn, true);
                }

                left = wheelSpeeds.left * Constants.kTopSpeedMPS;
                right = wheelSpeeds.right * Constants.kTopSpeedMPS;

                SmartDashboard.putNumber("leftvel" , left);
                SmartDashboard.putNumber("rightvel" , right);

                Drivetrain.setClosedLoop(left, right);
                break;

        }

    }

    // Because this Command is default, it never needs to end -- it will simply be
    // interrupted whenever another Command requires the drivetrain
    @Override
    public boolean isFinished() {
        return false;
    }

    // When this command ends, it stops the drivetrain to guarantee safety
    @Override
    public void end(boolean interrupted) {
        Drivetrain.clearLastVelocities();
        Drivetrain.setOpenLoop(0.0, 0.0);
    }

    protected static enum State {
        OpenLoop, CheesyDrive
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(requirements);
    }

}