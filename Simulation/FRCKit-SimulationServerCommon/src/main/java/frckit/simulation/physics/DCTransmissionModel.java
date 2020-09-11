package frckit.simulation.physics;

/**
 * Models the differential equation representation of a simple DC motor.
 * This is used to simulate the torque applied by the motor at each timestep given its velocity.
 */
public class DCTransmissionModel {
    private static final double EPSILON = 1e-6;

    private final double speedPerVolt;
    private final double torquePerVolt;
    private final double frictionVoltage;
    private final double ratio;

    public DCTransmissionModel(double speedPerVolt, double torquePerVolt, double efficiency, double nominalVoltage, int numMotors, double ratio) {
        this.speedPerVolt = speedPerVolt; //Speed remains the same with multiple motors
        this.torquePerVolt = torquePerVolt * numMotors; //Torque increases with added motors
        this.frictionVoltage = nominalVoltage - (nominalVoltage * efficiency); //Efficiency can be modelled by eating some of the input voltage
        this.ratio = ratio;
    }

    /**
     * Simulates the transmission's torque output given the input voltage and the rotor velocity
     * @param voltage Voltage applied to the transmission
     * @param velocity Current velocity of the transmission's output
     * @return Torque to be applied at the transmission's output
     */
    public double simulate(double voltage, double velocity) {
        double motorVel = velocity * ratio;
        double effectiveVoltage = voltage;
        if (motorVel > EPSILON) { //Motor spinning (dynamic) cases
            //Motor is spinning forwards, subtract friction voltage
            effectiveVoltage -= frictionVoltage;
        } else if (motorVel < -EPSILON) {
            //Motor is spinning backwards, add friction voltage
            effectiveVoltage += frictionVoltage;
        } else if (voltage > EPSILON) { //Motor stalled (static) cases
            //Motor is stalled, torque applied forwards
            effectiveVoltage = Math.max(0.0, voltage - frictionVoltage);
        } else if (voltage < -EPSILON) {
            //Motor is stalled, torque applied reverse
            effectiveVoltage = Math.min(0.0, voltage + frictionVoltage);
        } else { //Idle case
            //No voltage and no velocity, no torque
            return 0.0;
        }

        double motorTorque = torquePerVolt * (-motorVel / speedPerVolt + effectiveVoltage);
        return motorTorque * ratio;
    }
}
