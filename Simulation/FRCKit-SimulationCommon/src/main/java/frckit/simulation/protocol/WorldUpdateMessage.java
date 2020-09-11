package frckit.simulation.protocol;

import java.io.Serializable;

public class WorldUpdateMessage implements Serializable {
    public double timestamp;

    public final double[] transmission_positions;
    public final double[] transmission_velocities;

    public final double[] encoder_positions;
    public final double[] encoder_velocities;

    public WorldUpdateMessage(int numTransmissions, int numStandaloneEncoders) {
        transmission_positions = new double[numTransmissions];
        transmission_velocities = new double[numTransmissions];

        encoder_positions = new double[numStandaloneEncoders];
        encoder_velocities = new double[numStandaloneEncoders];
    }
}
