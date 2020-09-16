package frckit.simulation.control;

//This implementation of a PID controller is designed to match that of typical FRC motor controllers.
//The algorithm used closely matches the SPARK MAX PID control algorithm, which also matches the CTRE algorithm.
//Algorithm referenced from: https://www.revrobotics.com/sparkmax-users-manual/#section-3-4
public class PIDController {
    private double kP, kI, kD, kF, iZone; //Controller constants
    private double minOutput = Double.NEGATIVE_INFINITY; //Controller constraints
    private double maxOutput = Double.POSITIVE_INFINITY;
    private double i, prevError; //Controller state variables

    public double getkP() {
        return kP;
    }

    public void setkP(double kP) {
        this.kP = kP;
    }

    public double getkI() {
        return kI;
    }

    public void setkI(double kI) {
        this.kI = kI;
    }

    public double getkD() {
        return kD;
    }

    public void setkD(double kD) {
        this.kD = kD;
    }

    public double getkF() {
        return kF;
    }

    public void setkF(double kF) {
        this.kF = kF;
    }

    public double getiZone() {
        return iZone;
    }

    public void setiZone(double iZone) {
        this.iZone = iZone;
    }

    public double getMinOutput() {
        return minOutput;
    }

    public void setMinOutput(double minOutput) {
        this.minOutput = minOutput;
    }

    public double getMaxOutput() {
        return maxOutput;
    }

    public void setMaxOutput(double maxOutput) {
        this.maxOutput = maxOutput;
    }

    public double calculate(double setpoint, double pv) {
        double error = setpoint - pv;
        double p = error * kP;

        /*
        if (Math.abs(error) <= iZone || iZone == 0.0) {
            i = i + (error * kI);
        } else {
            i = 0.0;
        }

        double d = (error - prevError) * kD;
        prevError = error;

        double f = setpoint * kF;

        double output = p + i + d + f;

         */
        return p;
        //return Math.min(Math.max(output, minOutput), maxOutput); //Clamp the output
    }
}
