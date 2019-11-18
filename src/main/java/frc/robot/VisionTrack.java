package frc.robot;

import java.util.Set;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.Drivetrain.WheelState;
import frc.robot.Misc.Constants;
import frc.robot.Misc.OI;
import frc.robot.Misc.PDController;
import frc.robot.Misc.OI.LEDMode;
import frc.robot.Misc.OI.VisionPipeline;

public class VisionTrack implements Command {

    private double left, right;

    private double kPAim;
    private double kDAim;

    private double kPDist;
    private double kDDist;

    private PDController aim;
    private PDController dist;

    private double last_left = 0;
    private double last_right = 0;

    Subsystem[] requirements = {Robot.drivetrain};

    public VisionTrack() {

    }

    @Override
    public void initialize() {

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
    public void execute() {

        kPAim = SmartDashboard.getNumber("kPAim", Constants.kPAim);
        kDAim = SmartDashboard.getNumber("kDAim", Constants.kDAim);
        kPDist = SmartDashboard.getNumber("kPDist", Constants.kPDist);
        kDDist = SmartDashboard.getNumber("kDDist", Constants.kDDist);
        aim.updatePD(kPAim, kDAim);

        double aim_error = OI.getXOffset();
        double dist_error = OI.getYOffset();

        // Number of pixels error to tolerate (minimizes jittering when the error is
        // small)
        if (Math.abs(aim_error) < Constants.acceptablePixelError)
            aim_error = 0;
        if (Math.abs(dist_error) < Constants.acceptablePixelError)
            dist_error = 0;

        double aim_adjust = aim.calculate(aim_error); // Output of PD loop on heading
        double dist_adjust = dist.calculate(dist_error); // Output of PD loop on distance

        WheelState wheelspeeds;
        if (Math.abs(dist_adjust) > Constants.acceptableDistAdjustError) { // Quickturn once distance error is minimized
            wheelspeeds = Drivetrain.DifferentialDrive.curvatureDrive(dist_adjust, aim_adjust, false);
        } else {
            wheelspeeds = Drivetrain.DifferentialDrive.curvatureDrive(dist_adjust, aim_adjust, true);
        }

        left = wheelspeeds.left * Constants.kTopSpeedMPS;
        right = wheelspeeds.left * Constants.kTopSpeedMPS;

        /*
         * V_app = kS + kV * velocity + kA * acceleration; kS is multiplied by
         * signum(velocity), which returns 0 when desired velocity is 0
         */
        double leftFf = (Constants.kS * Math.signum(left) + Constants.kV * left
                + Constants.kA * (left - last_left) / 0.02) / 12;
        double rightFf = (Constants.kS * Math.signum(right) + Constants.kV * right
                + Constants.kA * (right - last_right) / 0.02) / 12;

        last_left = left;
        last_right = right;

        // Converting velocities to Talon native velocity units
        left = Drivetrain.DifferentialDrive.MPStoTicksPerDecisecond(left);
        right = Drivetrain.DifferentialDrive.MPStoTicksPerDecisecond(right);

        Drivetrain.setClosedLoop(left, leftFf, right, rightFf);

    }

    public boolean isFinished() {
        return false;
    }

    protected void end() {
        Drivetrain.setOpenloop(0, 0);
        Robot.oi.setPipeline(VisionPipeline.DRIVER);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(requirements);
    }
}