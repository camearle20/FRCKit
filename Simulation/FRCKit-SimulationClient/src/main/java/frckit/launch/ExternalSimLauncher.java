package frckit.launch;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.sim.SimHooks;
import edu.wpi.first.wpilibj.*;
import frckit.simulation.SimulationClient;
import net.bytebuddy.ByteBuddy;
import net.bytebuddy.agent.ByteBuddyAgent;
import net.bytebuddy.dynamic.loading.ClassReloadingStrategy;
import net.bytebuddy.implementation.MethodCall;
import net.bytebuddy.implementation.MethodDelegation;
import net.bytebuddy.matcher.ElementMatchers;

import java.lang.reflect.Method;
import java.util.concurrent.locks.ReentrantLock;

/**
 * This class allows users to launch robot code for use in an external simulation tool,
 * specifically one supported by one of FRCKit's "connectors".  Currently, Webots is supported.
 */
public class ExternalSimLauncher {
    private static class StopException extends RuntimeException {}

    private static final ReentrantLock runMutex = new ReentrantLock();
    private static RobotBase robotCopy;
    private static boolean suppressExitWarningGlobal;

    private static void receiveChecked(SimulationClient client) {
        try {
            client.receiveUpdateMessage();
        } catch (Exception e) {
            System.out.println("Simulation disconnected (" + e.getClass().getSimpleName() + ")");
            suppressExitWarningGlobal = true;
            throw new StopException();
        }
    }

    private static void sendChecked(SimulationClient client) {
        try {
            client.sendCycleMessage();
        } catch (Exception e) {
            System.out.println("Simulation disconnected (" + e.getClass().getSimpleName() + ")");
            suppressExitWarningGlobal = true;
            throw new StopException();
        }
    }

    public static class TimestampProxy {
        public static SimulationClient client;

        public static long getFPGATime() {
            return (long) (client.getLastWorldUpdate().getTimestamp() * 1000000);
        }

    }

    //helper called by "launch"
    private static void runRobot(Class<? extends IterativeRobotBase> robotClass, SimulationClient client) {
        System.out.println("********** Robot program starting **********");

        IterativeRobotBase robot;
        try {
            robot = robotClass.getConstructor().newInstance();
        } catch (Throwable throwable) {
            Throwable cause = throwable.getCause();
            if (cause != null) {
                throwable = cause;
            }
            String robotName = "Unknown";
            StackTraceElement[] elements = throwable.getStackTrace();
            if (elements.length > 0) {
                robotName = elements[0].getClassName();
            }
            DriverStation.reportError("Unhandled exception instantiating robot " + robotName + " "
                    + throwable.toString(), elements);
            DriverStation.reportWarning("Robots should not quit, but yours did!", false);
            DriverStation.reportError("Could not instantiate robot " + robotName + "!", false);
            return;
        }

        runMutex.lock();
        robotCopy = robot;
        runMutex.unlock();

        boolean errorOnExit = false;
        try {
            receiveChecked(client); //This blocks until webots is ready
            client.createNewCycleBuilder();
            robot.robotInit();
            HAL.observeUserProgramStarting();
            sendChecked(client);

            //Need to use reflection here to get access to the "loopFunc" method
            //of IterativeRobotBase.  We could reimplement it here, but that just
            //makes this more likely than it already is to break with future versions of WPILib
            //This is not ideal, but it works.
            Method loopFunc = IterativeRobotBase.class.getDeclaredMethod("loopFunc");
            loopFunc.setAccessible(true); //This removes the "protected" modifier, making it public

            //Run the code forever
            while (!Thread.interrupted()) {
                receiveChecked(client);
                client.createNewCycleBuilder(); //Create a new cycle message for this cycle
                client.setEnabled(robot.isEnabled()); //Report enabled state
                loopFunc.invoke(robot); //This is the same as "robot.loopFunc()" but using reflection
                sendChecked(client);
            }
        } catch (Throwable throwable) {
            if (!(throwable instanceof StopException)) {
                Throwable cause = throwable.getCause();
                if (cause != null) {
                    throwable = cause;
                }
                DriverStation.reportError("Unhandled exception: " + throwable.toString(),
                        throwable.getStackTrace());
                errorOnExit = true;
            }
        } finally {
            runMutex.lock();
            boolean suppressExitWarning = suppressExitWarningGlobal;
            runMutex.unlock();
            if (!suppressExitWarning) {
                // startCompetition never returns unless exception occurs....
                DriverStation.reportWarning("Robots should not quit, but yours did!", false);
                if (errorOnExit) {
                    DriverStation.reportError(
                            "The startCompetition() method (or methods called by it) should have "
                                    + "handled the exception above.", false);
                } else {
                    DriverStation.reportError("Unexpected return from startCompetition() method.", false);
                }
            }
        }
    }

    public static void launch(String serverAddress, int port, Class<? extends IterativeRobotBase> robotClass) {
        SimulationClient client = null;
        for (int i = 1; i <= 10; i++) {
            System.out.println("Connecting to simulation server at " + serverAddress + ":" + port + ", attempt " + i);
            try {
                client = new SimulationClient(serverAddress, port);
                break;
            } catch (Exception e) {
                System.err.println(e.getMessage());
            }
        }
        if (client == null) {
            System.err.println("Unable to connect to server.");
            return;
        }

        TimestampProxy.client = client;
        //Redefine timestamp
        ByteBuddyAgent.install();
        new ByteBuddy()
                .redefine(RobotController.class)
                .method(ElementMatchers.named("getFPGATime"))
                .intercept(MethodDelegation.to(TimestampProxy.class))
                .make()
                .load(RobotController.class.getClassLoader(), ClassReloadingStrategy.fromInstalledAgent());


        //A lot of this is referenced/copied from WPILib RobotBase, starting in "startRobot"
        //Things not needed in simulation (usage reporting, etc.) are skipped
        if (!HAL.initialize(500, 0)) {
            throw new IllegalStateException("Failed to initialize. Terminating");
        }

        if (HAL.hasMain()) {
            SimulationClient clientCopy = client;
            Thread thread = new Thread(() -> {
                runRobot(robotClass, clientCopy);
                HAL.exitMain();
            }, "robot main");
            thread.setDaemon(true);
            thread.start();
            HAL.runMain();
            runMutex.lock();
            suppressExitWarningGlobal = true;
            RobotBase robot = robotCopy;
            runMutex.unlock();
            if (robot != null) {
                robot.endCompetition();
            }
            try {
                thread.join(1000);
            } catch (InterruptedException ex) {
                Thread.currentThread().interrupt();
            }
        } else {
            runRobot(robotClass, client);
        }
    }
}
