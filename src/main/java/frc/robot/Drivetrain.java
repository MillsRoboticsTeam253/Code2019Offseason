package frc.robot;

import java.util.Arrays;

import javax.swing.text.StyleContext.SmallAttributeSet;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Auto.EncoderOdom;
import frc.robot.Misc.Constants;

public class Drivetrain extends Subsystem {

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

    // Odometry
    public EncoderOdom odometry = new EncoderOdom();

    // Will run the {@link Drive} Command when the subsystem is not otherwise being used
    public void initDefaultCommand() {
        setDefaultCommand(new Drive(Drive.State.CheesyDrive));
    }

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
    }

    /**
     * Sets the drivetrain to run at a set speed (open loop percent voltage or closed loop velocity)
     * 
     * @param mode Mode to use (same as TalonSRX set() modes)
     * @param left Value for left side of the drivetrain (in feet/sec for velocity mode)
     * @param right Value for right side of the drivetrain (in feet/sec for velocity mode)
     * @param type Secondary demand type
     * @param auxFf Secondary demand value (should be [-1, 1])
     */
    public static void setOpenloop(double left, double right){
        leftMotorA.set(ControlMode.PercentOutput, left);
        rightMotorA.set(ControlMode.PercentOutput, right);

    }

    public static void setClosedloop(double left, double leftFf, double right, double rightFf){
        leftMotorA.set(ControlMode.Velocity, left, DemandType.ArbitraryFeedForward, leftFf);
        rightMotorA.set(ControlMode.Velocity, right, DemandType.ArbitraryFeedForward, rightFf);

        SmartDashboard.putNumber("Left Control Effort", leftMotorA.getMotorOutputVoltage());
        SmartDashboard.putNumber("Right Control Effort", rightMotorA.getMotorOutputVoltage());

        SmartDashboard.putNumber("Left Error", leftMotorA.getClosedLoopError());
        SmartDashboard.putNumber("Right Error", rightMotorA.getClosedLoopError());
    }

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

    // Returns the current measurement of the left drivetrain encoder in feet, assuming 1024 encoder ticks per rotation and 4 inch diameter wheels
    public static double getLeftFeet(){
        return ((leftMotorA.getSelectedSensorPosition() / 1024.0) * 4*Math.PI) / 12;
    }

    // Returns the current measurement of the right drivetrain encoder in feet, assuming 1024 encoder ticks per rotation and 4 inch diameter wheels
    public static double getRightFeet(){
        return ((rightMotorA.getSelectedSensorPosition() / 1024.0) * 4*Math.PI) / 12;
    }

    public static double FPStoTicksPerDecisecond (double val) {
        return (((val/10.0)*12)/(4*Math.PI))*1024;
    }
}