package frckit.simulation.devices;

import frckit.simulation.SimulationGlobals;

public class SimTimer {
    private SimTimer() {} //no instances

    public static double getTimestampSeconds() {
        return SimulationGlobals.worldUpdate.timestamp;
    }
}
