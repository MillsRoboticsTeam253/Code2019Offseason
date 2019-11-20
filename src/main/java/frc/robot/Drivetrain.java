package frc.robot;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Misc.Constants;

public class Drivetrain implements Subsystem { 

    // Constructing speed controllers based on ID numbers
    public static TalonSRX leftMotorA = new TalonSRX(Constants.leftMotorA),
            leftMotorB = new TalonSRX(Constants.leftMotorB), rightMotorA = new TalonSRX(Constants.rightMotorA),
            rightMotorB = new TalonSRX(Constants.rightMotorB);
    private static VictorSPX leftMotorC = new VictorSPX(Constants.leftMotorC),
            rightMotorC = new VictorSPX(Constants.rightMotorC);

    // Creating arrays to make common settings easier
    private static IMotorController[] motors = { leftMotorA, leftMotorB, leftMotorC, rightMotorA, rightMotorB,
            rightMotorC };
    
    // This guarantees only one instance of Drivetrain should ever exist at once (prevents accidental runtime errors)
    private static Drivetrain instance = null;
    public static Drivetrain getInstance() {
        if (instance == null)
            instance = new Drivetrain();
        return instance;
    }

    // Closed loop kinematics
    public static SimpleMotorFeedforward motorFeedForward = new SimpleMotorFeedforward(Constants.kS, Constants.kV, Constants.kA);
    public static DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Constants.trackwidth);
    public static DifferentialDriveOdometry odometry;

    // Declaring variables to store acceleration values from last loop (to calculate acceleration)
    private static double last_left = 0;
    private static double last_right = 0;

    private static AHRS navX;

    private Drivetrain(){

        /* Setting follower and master speed controllers */
        leftMotorB.follow(leftMotorA);
        leftMotorC.follow(leftMotorA);
        rightMotorB.follow(rightMotorA);
        rightMotorC.follow(rightMotorA);

        /* Inversion on opposite sides of the drivetrain */
        Arrays.asList(leftMotorA, leftMotorB, leftMotorC).forEach(motor -> motor.setInverted(true));
        Arrays.asList(rightMotorA, rightMotorB, rightMotorC).forEach(motor -> motor.setInverted(false));

        /* Current limiting and voltage compensation settings */
        Arrays.stream(motors).forEach(motor -> {
            if (motor instanceof TalonSRX) {
                TalonSRX talon = (TalonSRX) motor;

                talon.configPeakCurrentLimit(45);
                talon.configPeakCurrentDuration(125);
                talon.configContinuousCurrentLimit(38);
                talon.enableCurrentLimit(true);
            }

            motor.configVoltageCompSaturation(12, 10);
            motor.enableVoltageCompensation(true);

            motor.setNeutralMode(NeutralMode.Brake);
        });

        /* Encoder settings */
        Arrays.asList(leftMotorA, rightMotorA).forEach(motor -> {
            motor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 1, 10);
            motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, 10);
        });
        leftMotorA.setSensorPhase(false);
        rightMotorA.setSensorPhase(false);

        leftMotorA.config_kP(0, Constants.kPVelocity);
        rightMotorA.config_kP(0, Constants.kPVelocity);

        odometry = new DifferentialDriveOdometry(kinematics, Rotation2d.fromDegrees(navX.getAngle()), new Pose2d());
    }

    /**
     * Handles the drivetrain's odometry and updating FalconDashboard
     */
    @Override
    public void periodic() {
        double leftVelocity = DifferentialDrive.TicksPerDecisecondtoMPS(Drivetrain.getLeftEncVelocity());
        double rightVelocity = DifferentialDrive.TicksPerDecisecondtoMPS(Drivetrain.getRightEncVelocity());

        odometry.update(Rotation2d.fromDegrees(navX.getAngle()), 
                        new DifferentialDriveWheelSpeeds(leftVelocity, rightVelocity));
        Robot.falcondashboard.putOdom(odometry.getPoseMeters());
    }

    /**
     * Sets odometry (used to set the position and angle offset at the beginning of autonomous)
     * 
     * @param poseMeters  Global pose of robot
     * @param gyroAngle   Angle robot is facing 
     */
    public void setOdometry(Pose2d poseMeters, Rotation2d gyroAngle){
        odometry.resetPosition(poseMeters, gyroAngle);
    }

    /**
     * Drives the robot open loop.
     * 
     * @param left   Percent of max output to set left side of drivetrain to [-1, 1]
     * @param right  Percent of max output to set right side of drivetrain to [-1, 1]
     */
    public static void setOpenLoop(Double left, Double right){
        leftMotorA.set(ControlMode.PercentOutput, left.doubleValue());
        rightMotorA.set(ControlMode.PercentOutput, right.doubleValue());
    }

    /**
     * Handles all kinematics for closed loop driving modes. 
     * 
     * @param left   Velocity to set left side of the drivetrain to (meters/second)
     * @param right  Velocity to set right side of the drivetrain to (meters/second)
     */
    public static void setClosedLoop(Double left, Double right) {
        double accelLeft = (left-last_left)/0.02;
        double accelRight = (right-last_right)/0.02;

        last_left = left;
        last_right = right;

        double leftff = motorFeedForward.calculate(left, accelLeft)/12;
        double rightff = motorFeedForward.calculate(right, accelRight)/12;

        left = DifferentialDrive.MPStoTicksPerDecisecond(left);
        right = DifferentialDrive.MPStoTicksPerDecisecond(right);

        setClosedLoop(left, leftff, right, rightff);
    }

    /**
     * Zeroes the last loop velocities used to calculate acceleration between closed loop driving modes
     */
    public static void clearLastVelocities(){
        last_left = 0;
        last_right = 0;
    }

    /**
     * Zeroes the encoders
     */
    public void resetEncoders(){
        Arrays.asList(leftMotorA, rightMotorA).forEach(motor -> motor.setSelectedSensorPosition(0));
    }

    // Returns the current measurement of the left drivetrain encoder
    public static double getLeftEnc(){
        return leftMotorA.getSelectedSensorPosition();
    }

    // Returns the current measurement of the right drivetrain encoder 
    public static double getRightEnc(){
        return rightMotorA.getSelectedSensorPosition();
    }

    // Returns the current velocity measurement of the left drivetrain encoder
    public static double getLeftEncVelocity() {
        return leftMotorA.getSelectedSensorVelocity();
    }

    // Returns the current velocity measurement of the right drivetrain encoder
    public static double getRightEncVelocity() {
        return rightMotorA.getSelectedSensorVelocity();
    }

    // Returns the current measurement of the left drivetrain encoder in feet, assuming 1024 encoder ticks per rotation and 4 inch diameter wheels
    public static double getLeftFeet(){
        return ((leftMotorA.getSelectedSensorPosition() / 1024.0) * 4*Math.PI) / 12;
    }

    // Returns the current measurement of the right drivetrain encoder in feet, assuming 1024 encoder ticks per rotation and 4 inch diameter wheels
    public static double getRightFeet(){
        return ((rightMotorA.getSelectedSensorPosition() / 1024.0) * 4*Math.PI) / 12;
    }

    /**
     * Function used to set the talon speeds in closed loop. Called by setClosedLoop(Double, Double)
     * after calculation of kinematics feedforwards and talon native speed units is complete
     * 
     * @param left     Velocity to set left side of drivetrain to in talon units per decisecond
     * @param leftFf   Feedforward for left side of drivetrain in Volts
     * @param right    Velocity to set right side of drivetrain to in talon units per decisecond
     * @param rightFf  Feedforward for right side of drivetrain in Volts
     */
    private static void setClosedLoop(double left, double leftFf, double right, double rightFf) {
        leftMotorA.set(ControlMode.Velocity, left, DemandType.ArbitraryFeedForward, leftFf);
        rightMotorA.set(ControlMode.Velocity, right, DemandType.ArbitraryFeedForward, rightFf);

        SmartDashboard.putNumber("Left Control Effort", leftMotorA.getMotorOutputVoltage());
        SmartDashboard.putNumber("Right Control Effort", rightMotorA.getMotorOutputVoltage());

        SmartDashboard.putNumber("Left Error", leftMotorA.getClosedLoopError());
        SmartDashboard.putNumber("Right Error", rightMotorA.getClosedLoopError());
    }

    // Static class to handle conversion from joystick inputs into left and right side outputs
    public static class DifferentialDrive {
        private static double quickStopAccumulator = 0.0;
        public static WheelState curvatureDrive(double linearPercent, double curvaturePercent, boolean isQuickTurn){
            final double angularPower;
            final boolean overPower;

            if (isQuickTurn){
                if(Math.abs(linearPercent) < Constants.kQuickStopThreshold) {
                    quickStopAccumulator = (1-Constants.kQuickStopAlpha) * quickStopAccumulator + 
                        Constants.kQuickStopAlpha * clamp(curvaturePercent, -1.0, 1.0) * 2.0;
                }
                overPower = true;
                angularPower = curvaturePercent;
                
            } else {
                overPower = false;
                angularPower = Math.abs(linearPercent) * curvaturePercent - quickStopAccumulator;
                
                if(quickStopAccumulator > 1) {
                    quickStopAccumulator -= 1.0;
                } else if(quickStopAccumulator < -1) {
                    quickStopAccumulator += 1.0;
                } else {
                    quickStopAccumulator = 0.0;
                }
            }

            double left = linearPercent + angularPower;
            double right = linearPercent - angularPower;

            SmartDashboard.putNumber("l_curv", left);
            SmartDashboard.putNumber("r_curv", right);


            // If rotation is overpowered, reduce both outputs to within acceptable range
            if (overPower) {
                if(left > 1) {
                    right -= left - 1;
                    left = 1;
                } else if (right > 1){
                    left -= right - 1;
                    right = 1;
                } else if (left < -1){
                    right -= left + 1;
                    left = -1;
                } else if (right < -1){
                    left -= right + 1;
                    right = -1;
                }
            }

            // Normalize the wheel speeds
            double maxMagnitude = Math.max(Math.abs(left), Math.abs(right));
            if(maxMagnitude > 1) {
                left /= maxMagnitude;
                right /= maxMagnitude;
            }

            return new WheelState(left, right);
        }

        private static double clamp(double val, double low, double high){
            return Math.max(low, Math.min(high, val));
        }

        public static double FPStoTicksPerDecisecond (double val) {
            return val * 0.00485104266 * Constants.ticksPerRotation / Constants.wheelRadius;
        }

        public static double MPStoTicksPerDecisecond (double val) {
            return val * 0.0159154943 * Constants.ticksPerRotation / Constants.wheelRadius;
        }

        public static double TicksPerDecisecondtoFPS (double val) {
            return val * 206.141250467 * Constants.wheelRadius / Constants.ticksPerRotation;
        }

        public static double TicksPerDecisecondtoMPS (double val) {
            return val * 62.8318530718 * Constants.wheelRadius / Constants.ticksPerRotation;
        }
    }

    // Static class to contain the speeds of each side of the drivetrain
    public static class WheelState {
        public double left, right;
        public WheelState(double left, double right){
            this.left = left;
            this.right = right;
        }
    }
    
}