package frckit.simulation.devices;

import frckit.simulation.SimulationClient;

public class SimTimer {
    private SimTimer() {} //no instances

    public static double getTimestampSeconds() {
        return SimulationClient.getInstance().getLastWorldUpdate().getTimestamp();
    }
}
