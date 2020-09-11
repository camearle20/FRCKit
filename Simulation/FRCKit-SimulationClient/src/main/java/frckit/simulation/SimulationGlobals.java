package frckit.simulation;

import frckit.simulation.protocol.RobotCycleMessage;
import frckit.simulation.protocol.WorldUpdateMessage;

/**
 * This class contains globals for incoming and outgoing simulation data.
 * While these can be used directly by user code, the recommended way of interacting
 * with the simulator is by using the appropriate device classes, found in the
 * "frckit.simulation.devices" package.
 */
public class SimulationGlobals {
    private SimulationGlobals() {} //no instances

    /**
     * The cycle message.  Robot code updates properties in this.
     */
    public static RobotCycleMessage cycleMessage = new RobotCycleMessage(0, 0);

    /**
     * The world update message.  The simulator updates properties in this.
     */
    public static WorldUpdateMessage worldUpdate;

    /**
     * Registers a new motor for the given slot.  If the slot number exceeds the number of currently allocated slots,
     * the method resizes the cycle message array to the given index + 1
     * @param idx The index of the motor
     */
    public static void registerMotor(int idx) {
        if (idx < 0) throw new RuntimeException("Invalid motor device index " + idx);
        int currentNumMotors = cycleMessage.motors_setpoints.length;
        int currentNumPneumatics = cycleMessage.pneumatics_setpoints.length;
        if (idx >= currentNumMotors) {
            //Create new object
            cycleMessage = new RobotCycleMessage(idx + 1, currentNumPneumatics);
        }
    }

    /**
     * Registers a new pneumatic device for the given slot.  If the slot number exceeds the number of currently allocated slots,
     * the method resizes the cycle message array to the given index + 1
     * @param idx The index of the pneumatic device
     */
    public static void registerPneumatic(int idx) {
        if (idx < 0) throw new RuntimeException("Invalid pneumatic device index " + idx);
        int currentNumPneumatics = cycleMessage.pneumatics_setpoints.length;
        int currentNumMotors = cycleMessage.motors_setpoints.length;
        if (idx >= currentNumPneumatics) {
            //Create new object
            cycleMessage = new RobotCycleMessage(currentNumMotors, idx + 1);
        }
    }
}
