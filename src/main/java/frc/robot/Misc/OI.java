package frc.robot.Misc;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

public class OI {
    private static XboxController xboxcontroller;
    private JoystickButton ButtonA;
    private JoystickButton ButtonB;
    private JoystickButton ButtonX;
    private JoystickButton ButtonY;
    private JoystickButton ButtonRB;
    private JoystickButton ButtonLB;
    private JoystickButton ButtonRT;
    private JoystickButton ButtonLT;

    private JoystickButton dpadUP;
    private JoystickButton dpadUP_RIGHT;
    private JoystickButton dpadRIGHT;
    private JoystickButton dpadDOWN_RIGHT;
    private JoystickButton dpadDOWN;
    private JoystickButton dpadDOWN_LEFT;
    private JoystickButton dpadLEFT;
    private JoystickButton dpadUP_LEFT;
    private JoystickButton dpadNONE;

    private JoystickButton triggerLeft;
    private JoystickButton triggerRight;

    private AHRS navX = new AHRS(Port.kMXP, (byte)200);

    private static OI instance = null;

    public static OI getInstance() {
        if (instance == null)
            instance = new OI();
        return instance;
    }

    public OI() {
        xboxcontroller = new XboxController(0);

        ButtonA = new JoystickButton(xboxcontroller, 1);
        ButtonB = new JoystickButton(xboxcontroller, 2);
        ButtonX = new JoystickButton(xboxcontroller, 3);
        ButtonY = new JoystickButton(xboxcontroller, 4);
        ButtonRB = new JoystickButton(xboxcontroller, 6);
        ButtonLB = new JoystickButton(xboxcontroller, 5);
        ButtonRT = new JoystickButton(xboxcontroller, 7);
        ButtonLT = new JoystickButton(xboxcontroller, 8);

        dpadUP = new XBPovButton(xboxcontroller, XBPovButton.UP);
        dpadUP_RIGHT = new XBPovButton(xboxcontroller, XBPovButton.UP_RIGHT);
        dpadRIGHT = new XBPovButton(xboxcontroller, XBPovButton.RIGHT);
        dpadDOWN_RIGHT = new XBPovButton(xboxcontroller, XBPovButton.DOWN_RIGHT);
        dpadDOWN = new XBPovButton(xboxcontroller, XBPovButton.DOWN);
        dpadDOWN_LEFT = new XBPovButton(xboxcontroller, XBPovButton.DOWN_LEFT);
        dpadLEFT = new XBPovButton(xboxcontroller, XBPovButton.LEFT);
        dpadUP_LEFT = new XBPovButton(xboxcontroller, XBPovButton.UP_LEFT);
        dpadNONE = new XBPovButton(xboxcontroller, XBPovButton.NONE);

        triggerLeft = new TriggerButton(xboxcontroller, Hand.kLeft);
        triggerRight = new TriggerButton(xboxcontroller, Hand.kRight);

    }

    public static double getThrottleValue() {
        // Controllers y-axes are natively up-negative, down-positive. returns negative
        return -deadbandX(xboxcontroller.getY(Hand.kLeft), Constants.kJoystickDeadband);
    }

    public static double getTurnValue() {
        return deadbandX(xboxcontroller.getX(Hand.kRight), Constants.kJoystickDeadband);
    }

    public static double deadbandX(double input, double deadband) {
        if (Math.abs(input) <= deadband) {
            return 0;
        } else if (Math.abs(input) == 1) {
            return input;
        } else {
            return (1 / (1 - deadband) * (input + Math.signum(-input) * deadband));
        }
    }

    public static double exponentiate(double input, double power) {
        return Math.copySign(Math.abs(Math.pow(input, power)), input);
    }

    public static double deadbandY(double input, double deadband) {
        if (Math.abs(input) == 0.0) {
            return 0;
        } else if (Math.abs(input) == 1) {
            return input;
        } else {
            return input * (1.0 - deadband) + Math.signum(input) * deadband;
        }
    }
}