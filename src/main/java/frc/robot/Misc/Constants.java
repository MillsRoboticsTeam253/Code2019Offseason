package frc.robot.Misc;

public class Constants {

    public static final int kPCM_ID = 1; // PCM ID

    /* Auto Settings */
    public static final double wheelbase = 1.583333333; // Wheelbase in feet

    /* Differential Drive Settings */
    public static final double kJoystickDeadband = 0.03; // Number between 0 and 1 representing how much of joystick is "dead" zone
    public static final double kTriggerDeadband = 0.05; // Number between 0 and 1 representing how much of trigger is "dead" zone

    public static final double kTurnSens = 0.86; // Maximum normal turning rate (in percent of max) to allow robot to turn to, b/t 0 and 1
    public static final double kDriveSens = 1; // Overall speed setting (turn down for demos)
    public static final double kTurnInPlaceSens = 0.8; // Maximum turn-in-place rate (in percent of max) to allow robot to turn to, b/t 0 and 1

    /* Cheesydrive Settings */
    public static final double kCurvatureScale = 30; // Turnrate in degrees/sec
    public static final double kV = 1.0; // Minimum voltage required to overcome static friction
    public static final double kTopSpeedFPS = 16.0;

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

}