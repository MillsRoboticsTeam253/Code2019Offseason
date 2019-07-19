package frc.robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Misc.Constants;
import frc.robot.Misc.OI;

public class Drive extends Command {

    private double left, right;
    
    public Drive(){
        
    }

    protected void execute(){
        double throttle = OI.getThrottleValue() * Constants.kDriveSens;
        double turn = OI.getTurnValue();

        SmartDashboard.putNumber("throttle", throttle);
        SmartDashboard.putNumber("turn", turn);

        if (throttle != 0) {
            left = throttle + throttle * turn * Constants.kTurnSens;
            right = throttle - throttle * turn * Constants.kTurnSens;

        } else {

            left = turn*Constants.kTurnInPlaceSens;
            right = -turn*Constants.kTurnInPlaceSens;

        }

        SmartDashboard.putNumber("left", left);
        SmartDashboard.putNumber("right", right);

        Drivetrain.set(true, left, right);
    }

    protected boolean isFinished() {
        return false;
    }

    protected void end() {
        Drivetrain.set(true, 0, 0);
    }
}