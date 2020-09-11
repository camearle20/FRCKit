package frckit.launch;

import com.google.inject.Guice;
import com.google.inject.Injector;
import com.google.inject.Module;
import com.google.inject.Stage;
import edu.wpi.first.wpilibj.IterativeRobotBase;
import edu.wpi.first.wpilibj.RobotBase;

import java.util.function.Supplier;

/**
 * This class enables launching the robot using WPILib with support for dependency injection.
 * An injector is created automatically, and is used to instantiate the robot class.  This permits
 * all classes used by the Robot class to be eligible for DI.
 */
public class RobotLauncher {
    private RobotLauncher() {} //no instances

    /**
     * Launches a robot program, using WPILib, with support for dependency injection
     * @param robotClass The Robot class
     * @param modules One or more Guice modules to register
     */
    public static void launch(Class<? extends IterativeRobotBase> robotClass, Module ... modules) {
        Injector injector = Guice.createInjector(Stage.PRODUCTION, modules);
        RobotBase.startRobot(() -> injector.getInstance(robotClass));
    }
}
