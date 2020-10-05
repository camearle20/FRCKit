package frckit.simulation;

import com.barchart.udt.OptionUDT;
import com.barchart.udt.net.NetSocketUDT;
import frckit.simulation.protocol.RobotCycle;
import frckit.simulation.protocol.WorldUpdate;

import java.io.*;
import java.net.InetSocketAddress;
import java.net.Socket;

public class SimulationClient {
    private final OutputStream out;
    private final InputStream in;
    private String name;

    //Store the last instance so that device classes can get access
    private static SimulationClient lastInstance = null;

    public static SimulationClient getInstance() {
        if (lastInstance == null) {
            throw new RuntimeException("No simulation client connected!  Did you use ExternalSimLauncher to launch your robot?");
        }
        return lastInstance;
    }

    /**
     * Returns true if the running instance of the JVM is connected to an external simulator.
     * @return True if the running code is connected to an external simulator, false otherwise.
     */
    public static boolean isExternalSim() {
        return lastInstance != null;
    }

    /**
     * Returns the name of the external simulator connected.
     * @return The name of the simulator connected, or an empty string if no simulator is connected or the connected
     * simulator is unnamed.
     */
    public static String getSimName() {
        if (lastInstance != null) {
            return lastInstance.name;
        } else {
            return "";
        }
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
            if (lastWorldUpdate.getSimulatorName() != null) {
                this.name = lastWorldUpdate.getSimulatorName();
            }
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
            NetSocketUDT socket = new NetSocketUDT();
            if (System.getProperty("os.name").contains("win"))
                socket.socketUDT().setOption(OptionUDT.UDT_MSS, 1052);
            socket.connect(new InetSocketAddress(serverAddress, serverPort));
            this.out = socket.getOutputStream();
            this.in = socket.getInputStream();
            lastInstance = this;
        } catch (IOException e) {
            throw new RuntimeException("Unable to connect to server at " + serverAddress + ":" + serverPort, e);
        }
    }
}
