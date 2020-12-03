package frckit.simulation.modelconfig;

import java.util.regex.Matcher;
import java.util.regex.Pattern;

/**
 * Full configuration for a simulation model.
 */
public class ModelConfiguration {
    public int robotCodeDtMs = 10;
    public int serverPort = 8889;
    public TransmissionConfig[] transmissions;
    public SensorConfig[] extraSensors;
    public int[] inertialUnits = {};

    public int getActualServerPort() {
        if (serverPort > 0) return serverPort;

        //Look for a number at the end of the username
        String username = System.getProperty("user.name");
        Pattern lastIntPattern = Pattern.compile("[^0-9]+([0-9]+)$");
        Matcher matcher = lastIntPattern.matcher(username);

        if (matcher.find()) {
            String number = matcher.group(1);
            return (-1 * serverPort) + Integer.parseInt(number);
        }

        return 0; //No port found
    }
}