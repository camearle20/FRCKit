package frckit.simulation;

import frckit.simulation.protocol.RobotCycleMessage;
import frckit.simulation.protocol.WorldUpdateMessage;

import java.io.IOException;
import java.io.ObjectInputStream;
import java.io.ObjectOutputStream;
import java.net.InetAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.SynchronousQueue;

/**
 * Implements the server which accepts incoming connection from robot code clients.  The server is configured
 * to accept input from only a single client at a time, and when a new connection is established, any previous connections
 * are closed and discarded.  This is to make it easy to quickly restart the robot code externally, as well as prevent
 * any lockup states from a forcibly closed connection.
 */
public class SimulationServer {
    private int port;
    private ClientHandler currentHandler;
    private BlockingQueue<WorldUpdateMessage> sendQueue = new SynchronousQueue<>();
    private BlockingQueue<RobotCycleMessage> recvQueue = new SynchronousQueue<>();

    private final Thread acceptClientsThread = new Thread(() -> {
        try {
            ServerSocket server = new ServerSocket(port);
            System.out.println("Accepting clients on port " + server.getLocalPort());
            while (!Thread.interrupted()) {
                Socket newClient = server.accept(); //Wait for a new client connection
                if (currentHandler != null && (currentHandler.socket.isConnected() || currentHandler.isAlive())) {
                    //A client is currently connected, disconnect them and then connect the new client
                    InetAddress oldAddr = currentHandler.socket.getInetAddress();
                    InetAddress newAddr = newClient.getInetAddress();
                    currentHandler.interrupt();
                    currentHandler.socket.close();
                    onClientChanged(oldAddr, newAddr);
                } else {
                    //No client is currently connected.
                    InetAddress addr = newClient.getInetAddress();
                    onNewClient(addr);
                }
                recvQueue.poll(); //Clear receive queue
                sendQueue.poll(); //Clear send queue
                recvQueue.put(RobotCycleMessage.WORLD_RESET);
                currentHandler = new ClientHandler(newClient); //Create and start the new client handler
                currentHandler.start();
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
        if (currentHandler != null) {
            currentHandler.interrupt();
        }
    });

    /**
     * Handles communication with individual clients.  There should only be one active instance of this class at a time.
     */
    private class ClientHandler extends Thread {
        final Socket socket;

        public ClientHandler(Socket socket) {
            this.socket = socket;
        }

        @Override
        public void run() {
            try {
                ObjectOutputStream out = new ObjectOutputStream(socket.getOutputStream());
                ObjectInputStream in = new ObjectInputStream(socket.getInputStream());

                while (!interrupted()) {
                    //Accept a world update from the queue
                    WorldUpdateMessage message = sendQueue.take();
                    //Send the message to the client
                    out.writeObject(message);
                    out.flush();
                    out.reset(); //Needed since ObjectOutputStream tracks reference ID and does not reserialize our reused instance each cycle
                    //Retrieve data from client
                    Object response = in.readObject();
                    if (response instanceof RobotCycleMessage) {
                        RobotCycleMessage cycleMessage = (RobotCycleMessage) response;
                        //Put cycle message in the queue
                        recvQueue.put(cycleMessage);
                    }
                }
            } catch (IOException | InterruptedException | ClassNotFoundException e) {
                e.printStackTrace();
            }
            if (!interrupted()) {
                //We reached this point because something went wrong with the client.  This constitutes a disconnect.
                onDisconnect(socket.getInetAddress());
            }
            currentHandler = null; //Delete the reference to this instance, should make this eligible for GC
        }
    }

    public SimulationServer(int port) throws IOException {
        this.port = port;
        acceptClientsThread.start();
    }

    /**
     * Called when an old client is disconnected by the server due to a new client requesting a connection.
     * @param oldAddr The IP address of the old client
     * @param newAddr The IP address of the new client
     */
    private void onClientChanged(InetAddress oldAddr, InetAddress newAddr) {
        if (oldAddr == null || newAddr == null) return;
        if (oldAddr.equals(newAddr)) {
            System.out.println("Client '" + oldAddr.getCanonicalHostName() + "' reconnected");
        } else {
            System.out.println("Client '" + oldAddr.getCanonicalHostName() + "' disconnected, client '" + newAddr.getCanonicalHostName() + "' connected");
        }
    }

    private void onNewClient(InetAddress addr) {
        if (addr == null) return;
        System.out.println("Client '" + addr.getCanonicalHostName() + "' connected");
    }

    private void onDisconnect(InetAddress addr) {
        if (addr == null) return;
        System.out.println("Client '" + addr.getCanonicalHostName() + "' disconnected");
    }

    public void sendWorldUpdate(WorldUpdateMessage message) throws InterruptedException {
        sendQueue.put(message);
    }

    public RobotCycleMessage getRobotCycleMessage() throws InterruptedException {
        return recvQueue.take();
    }

    public void stop() {
        acceptClientsThread.interrupt();
    }
}
