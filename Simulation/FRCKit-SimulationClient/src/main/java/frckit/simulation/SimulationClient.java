package frckit.simulation;

import frckit.simulation.protocol.RobotCycle;
import frckit.simulation.protocol.WorldUpdate;

import java.io.*;
import java.net.Socket;

public class SimulationClient {
    private final OutputStream out;
    private final InputStream in;

    //Store the last instance so that device classes can get access
    private static SimulationClient lastInstance = null;

    public static SimulationClient getInstance() {
        if (lastInstance == null) {
            throw new RuntimeException("No simulation client connected!  Did you use ExternalSimLauncher to launch your robot?");
        }
        return lastInstance;
    }

    private WorldUpdate.WorldUpdateMessage lastWorldUpdate = null;
    private RobotCycle.RobotCycleMessage.Builder currentCycleBuilder = null;

    public WorldUpdate.WorldUpdateMessage getLastWorldUpdate() {
        return lastWorldUpdate;
    }

    public RobotCycle.RobotCycleMessage.Builder getCurrentCycleBuilder() {
        return currentCycleBuilder;
    }

    public void receiveUpdateMessage() {
        try {
            //Get and store update from simulator
            lastWorldUpdate = WorldUpdate.WorldUpdateMessage.parseDelimitedFrom(in);
        } catch (IOException e) {
            throw new RuntimeException("Invalid message from server", e);
        }
    }

    public void createNewCycleBuilder() {
        //Create a new robotCycle message builder for this cycle
        currentCycleBuilder = RobotCycle.RobotCycleMessage.newBuilder();
    }

    public void setEnabled(boolean isEnabled) {
        currentCycleBuilder.setIsEnabled(isEnabled);
    }

    public void sendCycleMessage() {
        try {
            currentCycleBuilder.build().writeDelimitedTo(out);
        } catch (IOException e) {
            throw new RuntimeException("Error sending message to server", e);
        }
    }

    public SimulationClient(String serverAddress, int serverPort) {
        try {
            Socket socket = new Socket(serverAddress, serverPort);
            this.out = socket.getOutputStream();
            this.in = socket.getInputStream();
        } catch (IOException e) {
            throw new RuntimeException("Unable to connect to server", e);
        }
        lastInstance = this;
    }
}
