package frc.robot;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
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
        setDefaultCommand(new Drive());
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

                talon.configPeakCurrentLimit(60);
                talon.configPeakCurrentDuration(500);
                talon.configContinuousCurrentLimit(38);
                talon.enableCurrentLimit(false);
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
    }

    /**
     * Sets the drivetrain to run at a set speed (open loop percent voltage or closed loop velocity)
     * 
     * @param isOpenLoop Whether to use Percent Max Voltage or Velocity modes
     * @param left Value for left side of the drivetrain
     * @param right Value for right side of the drivetrain
     */
    public static void set(boolean isOpenLoop, double left, double right){
        if(isOpenLoop){
            leftMotorA.set(ControlMode.PercentOutput, left);
            rightMotorA.set(ControlMode.PercentOutput, right);
        } else {
            leftMotorA.set(ControlMode.Velocity, left);
            rightMotorA.set(ControlMode.Velocity, right);
        }
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
}