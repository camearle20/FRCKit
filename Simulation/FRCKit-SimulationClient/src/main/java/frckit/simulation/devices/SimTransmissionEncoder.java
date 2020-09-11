package frckit.simulation.devices;

import frckit.simulation.SimulationGlobals;

public class SimTransmissionEncoder {
    private int slot;

    public SimTransmissionEncoder(int slot) {
        this.slot = slot;
        SimulationGlobals.registerMotor(slot);
    }

    public double getPositionRadians() {
        return SimulationGlobals.worldUpdate.transmission_positions[slot];
    }

    public double getVelocityRadPerSec() {
        return SimulationGlobals.worldUpdate.transmission_velocities[slot];
    }
}
