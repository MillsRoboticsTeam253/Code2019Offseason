package frc.robot;

import java.util.Arrays;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.IMotorController;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;

import edu.wpi.first.wpilibj.command.Subsystem;
import frc.robot.Misc.Constants;

public class Drivetrain extends Subsystem {

    public static TalonSRX leftMotorA = new TalonSRX(Constants.leftMotorA),
            leftMotorB = new TalonSRX(Constants.leftMotorB), rightMotorA = new TalonSRX(Constants.rightMotorA),
            rightMotorB = new TalonSRX(Constants.rightMotorB);

    private static VictorSPX leftMotorC = new VictorSPX(Constants.leftMotorC),
            rightMotorC = new VictorSPX(Constants.rightMotorC);

    private static IMotorController[] motors = { leftMotorA, leftMotorB, leftMotorC, rightMotorA, rightMotorB,
            rightMotorC };

    private static Drivetrain instance = null;
    public static Drivetrain getInstance() {
        if (instance == null)
            instance = new Drivetrain();
        return instance;
    }

    public void initDefaultCommand() {
        //setDefaultCommand();
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

    public static void set(boolean isOpenLoop, double left, double right){
        if(isOpenLoop){
            leftMotorA.set(ControlMode.PercentOutput, left);
            rightMotorA.set(ControlMode.PercentOutput, right);
        } else {
            leftMotorA.set(ControlMode.Velocity, left);
            rightMotorA.set(ControlMode.Velocity, right);
        }
    }

    public static double getLeftEnc(){
        return leftMotorA.getSelectedSensorPosition();
    }

    public static double getRightEnc(){
        return rightMotorA.getSelectedSensorPosition();
    }
}