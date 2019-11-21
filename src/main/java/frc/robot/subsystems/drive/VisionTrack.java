package frc.robot.subsystems.drive;

import java.util.Set;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Robot;
import frc.robot.misc.Constants;
import frc.robot.misc.OI;
import frc.robot.misc.PDController;
import frc.robot.misc.OI.LEDMode;
import frc.robot.misc.OI.VisionPipeline;
import frc.robot.subsystems.drive.Drivetrain.WheelState;

public class VisionTrack implements Command {

    private double left, right;

    private double kPAim;
    private double kDAim;

    private double kPDist;
    private double kDDist;

    private PDController aim;
    private PDController dist;

    private VisionPipeline pipeline;

    Subsystem[] requirements = {Robot.drivetrain};

    public VisionTrack(VisionPipeline pipeline){
        this.pipeline = pipeline;
    }

    public VisionTrack(){
        this.pipeline = VisionPipeline.VISION;
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
        Robot.oi.setPipeline(pipeline); // Sets pipeline to vision pipeline
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

        Drivetrain.setClosedLoop(left, right);

    }

    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Drivetrain.clearLastVelocities();
        Drivetrain.setOpenLoop(0.0, 0.0);
        Robot.oi.setPipeline(VisionPipeline.DRIVER);
    }

    @Override
    public Set<Subsystem> getRequirements() {
        return Set.of(requirements);
    }
}
