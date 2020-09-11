package frckit.simulation.protocol;

import java.io.Serializable;

public class RobotCycleMessage implements Serializable {
    public enum MotorControlTypes {
        VOLTAGE,
        POSITION,
        VELOCITY
    }

    public boolean                   resetWorldFlag = false;
    public final MotorControlTypes[] motors_controlTypes;
    public final double[]            motors_setpoints; //Stores setpoints (velocity or position)
    public final double[]            motors_voltages; //Stores voltages (voltage mode and arb ff)
    public final boolean[]           pneumatics_setpoints;

    public RobotCycleMessage(int numMotors, int numPneumatics) {
        motors_controlTypes = new MotorControlTypes[numMotors];
        motors_setpoints = new double[numMotors];
        motors_voltages = new double[numMotors];
        pneumatics_setpoints = new boolean[numPneumatics];
    }

    //Static instance to use to indicate world reset
    public static final RobotCycleMessage WORLD_RESET = new RobotCycleMessage(0, 0);
    static {
        WORLD_RESET.resetWorldFlag = true;
    }
}
