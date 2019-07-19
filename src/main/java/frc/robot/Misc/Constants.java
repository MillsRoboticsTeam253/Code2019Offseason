package frc.robot.Misc;

public class Constants {

    public static final int kPCM_ID = 1; // PCM ID

    /* Driver Settings */
    public static final double kJoystickDeadband = 0.03; // Number between 0 and 1 representing how much of joystick is "dead" zone
    public static final double kTriggerDeadband = 0.05; // Number between 0 and 1 representing how much of trigger is "dead" zone

    public static final double kTurnSens = 0.86; // Maximum normal turning rate (in percent of max) to allow robot to turn to, b/t 0 and 1
    public static final double kDriveSens = 1; // Overall speed setting (turn down for demos)
    public static final double kTurnInPlaceSens = 0.8; // Maximum turn-in-place rate (in percent of max) to allow robot to turn to, b/t 0 and 1

    /* Drivetrain Motor IDs */
    public static final int leftMotorA = 1;//TalonSRX
    public static final int leftMotorB = 2;//TalonSRX
    public static final int leftMotorC = 1;//VictorSPX

    public static final int rightMotorA = 3;//TalonSRX
    public static final int rightMotorB = 4;//TalonSRX
    public static final int rightMotorC = 2;//VictorSPX


}