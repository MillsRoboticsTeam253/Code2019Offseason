package frc.robot.Misc;

public class Constants {

    public static final int kPCM_ID = 1; // PCM ID

    /* Auto Settings */
    public static final double wheelbase = 1.7083333333; // Wheelbase in feet

    /* Differential Drive Settings */
    public static final double kJoystickDeadband = 0.03; // Number between 0 and 1 representing how much of joystick is "dead" zone
    public static final double kTriggerDeadband = 0.05; // Number between 0 and 1 representing how much of trigger is "dead" zone

    public static final double kTurnSens = 1; // Maximum normal turning rate (in percent of max) to allow robot to turn to, b/t 0 and 1
    public static final double kDriveSens = 1; // Overall speed setting (turn down for demos)
    public static final double kTurnInPlaceSens = 1; // Maximum turn-in-place rate (in percent of max) to allow robot to turn to, b/t 0 and 1

    /* Cheesydrive Settings */
    public static final double kS = 1.33; // Minimum voltage required to overcome static friction
    public static final double kV = 0.601;
    public static final double kA = 0.158;
    public static final double kTopSpeedFPS = 16.0;
    public static final double kPVelocity = 11.4;

    // Magic WPILib numbers for cheesydrive
    public static final double kQuickStopThreshold = 0.2;
    public static final double kQuickStopAlpha = 0.1;

    // Transmission Models 
    public static final double kSpeedPerVolt = 0; // rad/s per volt
    public static final double kTorquePerVolt = 0; // Nm per volt
    public static final double kFrictionVoltage = 0; // V

    /* Drivetrain Motor IDs */
    public static final int leftMotorA = 1; // TalonSRX
    public static final int leftMotorB = 2; // TalonSRX
    public static final int leftMotorC = 1; // VictorSPX

    public static final int rightMotorA = 3; // TalonSRX
    public static final int rightMotorB = 4; // TalonSRX
    public static final int rightMotorC = 2; // VictorSPX

    /* Vision PD Controller Gains */
    public static final int kPAim = 0;
    public static final int kDAim = 0;

    public static final int kPDist = 0;
    public static final int kDDist = 0;

    public static final int acceptableError = 1;
    public static final double acceptableDistAdjustError = 0.05;

}