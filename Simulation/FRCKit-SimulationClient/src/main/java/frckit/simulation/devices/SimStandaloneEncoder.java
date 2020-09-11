package frckit.simulation.devices;

import frckit.simulation.SimulationGlobals;

public class SimStandaloneEncoder {
    private int slot;

    public SimStandaloneEncoder(int slot) {
        this.slot = slot;
    }

    public double getPositionRadians() {
        return SimulationGlobals.worldUpdate.encoder_positions[slot];
    }

    public double getVelocityRadPerSec() {
        return SimulationGlobals.worldUpdate.encoder_velocities[slot];
    }
}
