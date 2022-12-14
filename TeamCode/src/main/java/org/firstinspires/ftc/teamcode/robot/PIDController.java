package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

// Proportional-Integral-Derivative Controller class for making movement smoother
public class PIDController {

    public double k_P; // Proportional coefficient
    public double k_I; // Integral coefficient
    public double k_D; // Derivative coefficient

    private double p_error; // proportional error
    private double i_error; // integral error
    private double d_error; // derivative error

    private double toleranceRadius; // PID won't adjust within this range

    // Minimum value the PID can return
    // Useful if the robot is heavy and won't move below a certain power threshold
    private double minAbsVal;

    private double min; // Minimum possible value this PID can return
    private double max; // Maximum possible value the PID can return

    private double prevError; // previous p_error
    private double prevTime; // measured in seconds!

    private ElapsedTime time; // For keeping track of time passed

    // "Empty" PID that only returns 0 and does basically nothing
    // Useful if you need to avoid NullPointerExceptions in certain instances
    public PIDController() {
        this(0, 0, 0, 0, 0, 0, 0);
    }

    // Default values for min and max are -1.0 and 1.0, respectively
    public PIDController(double Kp, double Ki, double Kd, double tolerance) {
        this(Kp, Ki, Kd, tolerance, 0, -1.0, 1.0);
    }

    public PIDController(double Kp, double Ki, double Kd, double tolerance, double minAbsVal) {
        this(Kp, Ki, Kd, tolerance, minAbsVal, -1.0, 1.0);
    }

    public PIDController(double Kp, double Ki, double Kd, double tolerance, double min, double max) {
        this(Kp, Ki, Kd, tolerance, 0, min, max);
    }

    // Constructs a PID object that regulates a quantity (typically motor powers)
    public PIDController(double Kp, double Ki, double Kd, double tolerance, double minAbsVal, double min, double max) {
        k_P = Kp;
        k_I = Ki; //probably zero or negative
        k_D = Kd;
        toleranceRadius = tolerance;
        this.minAbsVal = minAbsVal;
        this.min = min;
        this.max = max;
        time = new ElapsedTime();
        resetValues();
    }

    // Calculates the power value to be sent to the motor(s)
    public double calcVal(double error) {
        return calcVal(error, minAbsVal);
    }

    // Calculates the power value to be sent to the motor(s) with a specified lower bound for the abs val
    public double calcVal(double error, double minAbsVal) {

        // Calculates the different errors
        double deltaTime = time.seconds() - prevTime;
        p_error = error; //Current dist from target => x
        i_error += error * deltaTime; //How much distance you expect to gain by next time step => x * h
        d_error = (error - prevError) / deltaTime; //Rate of change of your displacement (negative) => dx/dt

        // Updates the "prev" variables for the next loop
        prevError = error;
        prevTime = time.seconds();

        // If the error is small enough, the robot won't adjust
        if (Math.abs(error) <= toleranceRadius) {
            return 0;
        }

        // Calculates the PID value to be sent to the motor
        double val = Range.clip(k_P * p_error + k_I * i_error + k_D * d_error, min, max);
        if (minAbsVal > 0) {
            val = Math.signum(val) * Math.max(minAbsVal, Math.abs(val));
        }
        return val;
    }

    // Resets the PID controller
    // Use this between longer interruptions of use
    public void resetValues() {
        p_error = 0;
        i_error = 0;
        d_error = 0;
        prevError = 0;
        prevTime = 0;
        time.reset();
    }

    // Change the min and max
    // Useful if you want the robot to drive more slowly in auto, for instance
    public void setMinMax(double min, double max) {
        this.min = min;
        this.max = max;
    }
}