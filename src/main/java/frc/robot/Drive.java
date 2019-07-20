package frc.robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Misc.Constants;
import frc.robot.Misc.OI;

public class Drive extends Command {

    private double left, right;

    public Drive() {
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

        // Cheesy drive as long as throttle is greater than zero (deadbanded). 
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

        Drivetrain.set(true, left, right);
    }

    // Because this Command is default, it never needs to end -- it will simply be interrupted whenever another Command requires the drivetrain
    protected boolean isFinished() {
        return false;
    }

    // When this command ends, it stops the drivetrain to guarantee safety
    protected void end() {
        Drivetrain.set(true, 0, 0);
    }
}