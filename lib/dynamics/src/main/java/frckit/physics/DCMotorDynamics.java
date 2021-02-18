package frckit.physics;

/**
 * Model of a DC transmission
 */
public class DCMotorDynamics {
    private final double speedPerVolt;
    private final double torquePerVolt;
    private final double frictionVoltage;

    /**
     * Creates a new DCMotorDynamics
     * @param speedPerVolt The speed per volt in rad/s / V
     * @param torquePerVolt The torque per volt in N*m / V
     * @param frictionVoltage The friction voltage (drag voltage) in V
     */
    public DCMotorDynamics(double speedPerVolt, double torquePerVolt, double frictionVoltage) {
        this.speedPerVolt = speedPerVolt;
        this.torquePerVolt = torquePerVolt;
        this.frictionVoltage = frictionVoltage;
    }

    /**
     * @return The speed per volt constant in rad/s / V
     */
    public double getSpeedPerVolt() {
        return speedPerVolt;
    }

    /**
     * @return The torque per volt in N*m / V
     */
    public double getTorquePerVolt() {
        return torquePerVolt;
    }

    /**
     * @return The friction voltage (drag voltage) in V
     */
    public double getFrictionVoltage() {
        return frictionVoltage;
    }

    /**
     * Calculates the free speed of the motor at a given voltage.
     * Accounts for drag voltage on the motor (static friction)
     * @param voltage The voltage applied
     * @return The free speed
     */
    public double getFreeSpeedAtVoltage(double voltage) {
        if (voltage > 1e-12) { //Forwards voltage
            return Math.max(0.0, voltage - frictionVoltage) * speedPerVolt;
        } else if (voltage < -1e-12) { //Reverse voltage
            return Math.min(0.0, voltage + frictionVoltage) * speedPerVolt;
        } else { //No voltage
            return 0.0;
        }
    }

    /**
     * Performs forward dynamics, calculating the torque produced by the motor at the given voltage
     * @param speed The rotational speed of the motor
     * @param voltage The applied voltage
     * @return The torque that is being produced by the motor
     */
    public double forwardDynamics(double speed, double voltage) {
        double effectiveVoltage = voltage;
        if (speed > 1e-12) {
            // Forward motion, rolling friction.
            effectiveVoltage -= frictionVoltage;
        } else if (speed < -1e-12) {
            // Reverse motion, rolling friction.
            effectiveVoltage += frictionVoltage;
        } else if (voltage > 1e-12) {
            // System is static, forward torque.
            effectiveVoltage = Math.max(0.0, voltage - frictionVoltage);
        } else if (voltage < -1e-12) {
            // System is static, reverse torque.
            effectiveVoltage = Math.min(0.0, voltage + frictionVoltage);
        } else {
            // System is idle.
            return 0.0;
        }
        return torquePerVolt * (-speed / speedPerVolt + effectiveVoltage);
    }

    /**
     * Performs inverse dynamics, calculating the voltage required to apply to the motor which yields the desired torque
     * @param speed The rotational speed of the motor
     * @param torque The desired torque
     * @return The voltage required to produce the torque
     */
    public double inverseDynamics(double speed, double torque) {
        double effectiveFrictionVoltage;
        if (speed > 1e-12) {
            // Forward motion, rolling friction.
            effectiveFrictionVoltage = frictionVoltage;
        } else if (speed < -1e-12) {
            // Reverse motion, rolling friction.
            effectiveFrictionVoltage = -frictionVoltage;
        } else if (torque > 1e-12) {
            // System is static, forward torque.
            effectiveFrictionVoltage = frictionVoltage;
        } else if (torque < -1e-12) {
            // System is static, reverse torque.
            effectiveFrictionVoltage = -frictionVoltage;
        } else {
            // System is idle.
            return 0.0;
        }
        return torque / torquePerVolt + speed / speedPerVolt + effectiveFrictionVoltage;
    }
}
