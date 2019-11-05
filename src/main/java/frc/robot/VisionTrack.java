package frc.robot;

import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.Misc.Constants;
import frc.robot.Misc.OI;
import frc.robot.Misc.PDController;
import frc.robot.Misc.OI.LEDMode;
import frc.robot.Misc.OI.VisionPipeline;

public class VisionTrack extends Command {

    private double left, right;

    private double kPAim;
    private double kDAim;

    private double kPDist;
    private double kDDist;

    private PDController aim;
    private PDController dist;

    private double last_left = 0;
    private double last_right = 0;

    public VisionTrack(){
        requires(Robot.drivetrain);

    }

    @Override
    protected void initialize(){

        kPAim = Constants.kPAim;
        kDAim = Constants.kDAim;
        kPDist = Constants.kPDist;
        kDDist = Constants.kDDist;

        SmartDashboard.putNumber("kPAim", kPAim);
        SmartDashboard.putNumber("kDAim", kDAim);
        SmartDashboard.putNumber("kPDist", kPDist);
        SmartDashboard.putNumber("kDDist", kDDist);

        Robot.oi.setLEDMode(LEDMode.ON); // Turns on Limelight LEDs
        Robot.oi.setPipeline(VisionPipeline.VISION); // Sets pipeline to vision pipeline
        Robot.oi.setCamMode(); // Sets camera to vision mode 

    }

    @Override
    protected void execute() {

        kPAim = SmartDashboard.getNumber("kPAim", Constants.kPAim);
        kDAim = SmartDashboard.getNumber("kDAim", Constants.kDAim);
        kPDist = SmartDashboard.getNumber("kPDist", Constants.kPDist);
        kDDist = SmartDashboard.getNumber("kDDist", Constants.kDDist);
        aim.updatePD(kPAim, kDAim);

        double aim_error = OI.getXOffset(); 
        double dist_error = OI.getYOffset();

        if(Math.abs(aim_error) < Constants.acceptableError) aim_error = 0;
        if(Math.abs(dist_error) < Constants.acceptableError) dist_error = 0;

        double aim_adjust = aim.calculate(aim_error); // Output of PD loop on heading
        double dist_adjust = dist.calculate(dist_error); // Output of PD loop on distance

        /*
        aim_adjust = clamp(aim_adjust, -1, 1);
        dist_adjust = clamp(dist_adjust, -1, 1);

        left = dist_adjust + Math.abs(dist_adjust) * aim_adjust; 
        right = dist_adjust - Math.abs(dist_adjust) * aim_adjust;

        // Normalize speeds
        double maxMagnitude = Math.max(Math.abs(left), Math.abs(right));
        if(maxMagnitude > 1) {
            left = left / maxMagnitude;
            right = right / maxMagnitude;
        }*/

        aim_adjust = Math.toRadians(aim_adjust*Constants.kCurvatureScale); // Converting to radian units, with arbitrary rescale factor
        dist_adjust = dist_adjust*Constants.kTopSpeedFPS; // Converting to velocity units

        left = dist_adjust - (Constants.wheelbase / 2.0) * aim_adjust;
        right = dist_adjust + (Constants.wheelbase / 2.0) * aim_adjust;

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

        SmartDashboard.putNumber("Vision left", left);
        SmartDashboard.putNumber("Vision right", right);
        
    }

    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void interrupted(){
        end();
    }

    protected void end() {
        Drivetrain.setOpenloop(0, 0);
        Robot.oi.setPipeline(VisionPipeline.DRIVER);
    }

    private double clamp(double val, double lower, double higher) {
        return Math.max(lower, Math.min(val, higher));
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