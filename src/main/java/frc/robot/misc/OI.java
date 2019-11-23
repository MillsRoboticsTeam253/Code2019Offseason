package frc.robot.misc;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.SPI.Port;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import frc.robot.Robot;

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

    private static AHRS navX;
    private static NetworkTable limelight;

    private static OI instance = null;

    public static OI getInstance() {
        if (instance == null)
            instance = new OI();
        return instance;
    }

    public OI() {
        navX = Robot.navX;
        limelight = NetworkTableInstance.getDefault().getTable("limelight");
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
        // Controllers y-axes are natively up-negative, down-positive. This method corrects that by returning the opposite of the value
        return -deadbandX(xboxcontroller.getY(Hand.kLeft), Constants.kJoystickDeadband);
    }

    public static double getTurnValue() {
        return deadbandX(xboxcontroller.getX(Hand.kRight), Constants.kJoystickDeadband);
    }

    public static double getGyroDegrees(){
        return navX.getFusedHeading();
    }

    public static double getGyroRadians(){
        double rad = navX.getFusedHeading() * Math.PI/180.0;
        rad = -rad; // Making the angle measurement CCW Positive (navX natively CW Positive)

        return rad;
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

     /*
     * Methods for setting limelight values
     */

    public void setLEDMode(LEDMode ledMode) {
        limelight.getEntry("ledMode").setNumber(ledMode.val);
    }

    public void setCamMode() {
        limelight.getEntry("camMode").setNumber(0);
    }

    public void setStreamMode(StreamMode stream) {
        limelight.getEntry("stream").setNumber(stream.val);
    }

    public void setPipeline(VisionPipeline pipeline) {
        limelight.getEntry("pipeline").setNumber(pipeline.val);
    }

    public static double getXOffset() {
        return -limelight.getEntry("tx").getDouble(0);
    }

    public static double getYOffset() {
        return -limelight.getEntry("ty").getDouble(0.0);
    }

    public static enum LEDMode {
        PIPELINE(0), OFF(1), BLINK(2), ON(3);

        public int val;

        private LEDMode(int val) {
            this.val = val;
        }
    }

    public static enum StreamMode {
        SIDE_BY_SIDE(0), PIP_MAIN(1), PIP_SECONDARY(2);

        public int val;
        
        private StreamMode(int val){
            this.val = val;
        }
    }

    public static enum VisionPipeline {
        VISION(0), DRIVER(1);

        public int val;

        private VisionPipeline(int val){
            this.val = val;
        }
    }
}