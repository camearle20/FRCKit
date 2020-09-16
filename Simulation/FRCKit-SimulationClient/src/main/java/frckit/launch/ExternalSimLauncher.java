package frckit.launch;

import com.google.inject.Module;
import com.google.inject.Guice;
import com.google.inject.Injector;
import com.google.inject.Stage;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.RobotBase;
import frckit.simulation.SimulationClient;

import java.lang.reflect.Method;
import java.util.concurrent.CopyOnWriteArrayList;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Supplier;

/**
 * This class allows users to launch robot code for use in an external simulation tool,
 * specifically one supported by one of FRCKit's "connectors".  Currently, Webots is supported.
 */
public class ExternalSimLauncher {
    private static final ReentrantLock runMutex = new ReentrantLock();
    private static RobotBase robotCopy;
    private static boolean suppressExitWarningGlobal;

    //helper called by "launch"
    private static void runRobot(Injector injector, Class<? extends IterativeRobotBase> robotClass, SimulationClient client) {
        System.out.println("********** Robot program starting **********");

        IterativeRobotBase robot;
        try {
            robot = injector.getInstance(robotClass);
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
            client.receiveUpdateMessage(); //This blocks until webots is ready
            client.createNewCycleBuilder();
            robot.robotInit();
            HAL.observeUserProgramStarting();
            client.sendCycleMessage();

            //Need to use reflection here to get access to the "loopFunc" method
            //of IterativeRobotBase.  We could reimplement it here, but that just
            //makes this more likely than it already is to break with future versions of WPILib
            //This is not ideal, but it works.
            Method loopFunc = IterativeRobotBase.class.getDeclaredMethod("loopFunc");
            loopFunc.setAccessible(true); //This removes the "protected" modifier, making it public

            //Run the code forever
            while (!Thread.interrupted()) {
                client.receiveUpdateMessage();
                client.createNewCycleBuilder(); //Create a new cycle message for this cycle
                client.setEnabled(robot.isEnabled()); //Report enabled state
                loopFunc.invoke(robot); //This is the same as "robot.loopFunc()" but using reflection
                client.sendCycleMessage(); //Send the response
            }
        } catch (Throwable throwable) {
            Throwable cause = throwable.getCause();
            if (cause != null) {
                throwable = cause;
            }
            DriverStation.reportError("Unhandled exception: " + throwable.toString(),
                    throwable.getStackTrace());
            errorOnExit = true;
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

    public static void launch(String serverAddress, int port, Class<? extends IterativeRobotBase> robotClass, Module ... modules) {
        Injector injector = Guice.createInjector(Stage.PRODUCTION, modules);
        SimulationClient client = new SimulationClient(serverAddress, port);

        //A lot of this is referenced/copied from WPILib RobotBase, starting in "startRobot"
        //Things not needed in simulation (usage reporting, etc.) are skipped
        if (!HAL.initialize(500, 0)) {
            throw new IllegalStateException("Failed to initialize. Terminating");
        }

        if (HAL.hasMain()) {
            Thread thread = new Thread(() -> {
                runRobot(injector, robotClass, client);
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
            runRobot(injector, robotClass, client);
        }
    }
}
