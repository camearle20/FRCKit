package frckit.simulation.modelconfig;

/**
 * Full configuration for a simulation model.
 */
public class ModelConfiguration {
    public int serverPort = 8889;
    public TransmissionConfig[] transmissions;
    public SensorConfig[] extraSensors;
}
