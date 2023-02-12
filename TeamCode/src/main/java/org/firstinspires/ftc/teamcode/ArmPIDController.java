package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.util.ElapsedTime;

// Single use per object
public class ArmPIDController {
    private final double kP;
    private final double kI;
    private final double kD;
    private final ElapsedTime timer;
    private double targetPosition;
    private double lastError = 0;
    private double accumulatedError = 0;
    private double lastTime = -1;
    private double lastDerivative = 0;

    private double power = 0;

    public ArmPIDController(double target, double p, double i, double d, ElapsedTime runtime) {
        kP = p;
        kI = i;
        kD = d;
        targetPosition = target;
        timer = runtime;
    }

    public void updatePower(double currentPosition) {
        // P
        double error = targetPosition - currentPosition;

        // I
        accumulatedError *= Math.signum(error);
        accumulatedError += error;
        if (Math.abs(error) < 5) {
            accumulatedError = 0;
        }

        // D
        double derivative = 0;
        if (lastTime > 0) {
            derivative = (error - lastError) / (timer.milliseconds() - lastTime);
        }
        lastDerivative = derivative;
        lastError = error;
        lastTime = timer.milliseconds();

        power = 0.1 * Math.signum(error)
                + 0.9 * Math.tanh(kP * error + kI * accumulatedError - kD * lastDerivative);
    }

    public void updateTarget(double newTargetPosition) {
        targetPosition = newTargetPosition;
    }

    public double getLastSlope() {
        return lastDerivative;
    }

    public double getPower() {return power;}
}