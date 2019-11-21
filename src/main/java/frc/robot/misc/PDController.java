package frc.robot.misc;

public class PDController {

    private double kP, kD, dt;
    private double lastError = 0;
    private boolean firstLoop = true;

    public PDController(double kP, double kD, double dt) {
        this.kP = kP;
        this.kD = kD;
        this.dt = dt;
        
    }

    public void updatePD(double kP, double kD){
        this.kP = kP;
        this.kD = kD;
    }

    public double calculate(double error) {

        double correction;

        if (firstLoop) { // No kD on first loop to prevent sudden movement from high change in error
            correction = kP * error;
            this.lastError = error;
            firstLoop = false;
        } else {
            correction = kP * error + kD * (error - lastError) / dt;
            this.lastError = error;
        }

        return correction;
    }
}