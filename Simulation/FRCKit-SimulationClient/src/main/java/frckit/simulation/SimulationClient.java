package frckit.simulation;

import frckit.simulation.protocol.RobotCycleMessage;
import frckit.simulation.protocol.WorldUpdateMessage;

import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.net.Socket;

public class SimulationClient {
    private final ObjectOutputStream out;
    private final ObjectInputStream in;

    public WorldUpdateMessage receiveUpdateMessage() {
        try {
            Object update = in.readObject();
            if (update instanceof WorldUpdateMessage) {
                return (WorldUpdateMessage) update;
            } else {
                throw new RuntimeException("Invalid message type '" + update.getClass().getName() + "' from server");
            }
        } catch (IOException | ClassNotFoundException e) {
            throw new RuntimeException("Invalid message from server", e);
        }
    }

    public void sendCycleMessage(RobotCycleMessage message) {
        try {
            out.writeObject(message);
            out.flush();
            out.reset();
        } catch (IOException e) {
            throw new RuntimeException("Error sending message to server", e);
        }
    }

    public SimulationClient(String serverAddress, int serverPort) {
        try {
            Socket socket = new Socket(serverAddress, serverPort);
            this.out = new ObjectOutputStream(socket.getOutputStream());
            this.in = new ObjectInputStream(socket.getInputStream());
        } catch (IOException e) {
            throw new RuntimeException("Unable to connect to server", e);
        }
    }
}
