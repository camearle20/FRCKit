package frckit.physics.state;

import edu.wpi.first.wpilibj.geometry.Twist2d;
import frckit.util.GeomUtil;

/**
 * Represents the state of a rigid body, such as a drivetrain or other mechanism, in terms of its velocity and acceleration
 */
public class RigidBodyState2d {
    public static final RigidBodyState2d ZERO = new RigidBodyState2d();

    private final double vx;
    private final double vy;
    private final double omega;
    private final double ax;
    private final double ay;
    private final double alpha;

    public RigidBodyState2d(double vx, double vy, double omega, double ax, double ay, double alpha) {
        this.vx = vx;
        this.vy = vy;
        this.omega = omega;
        this.ax = ax;
        this.ay = ay;
        this.alpha = alpha;
    }

    private RigidBodyState2d() {
        this(0.0, 0.0, 0.0, 0.0, 0.0, 0.0);
    }

    /**
     * @return The x component of the linear velocity of the body
     */
    public double getVx() {
        return vx;
    }

    /**
     * @return The y component of the linear velocity of the body
     */
    public double getVy() {
        return vy;
    }

    /**
     * @return The angular velocity of the body
     */
    public double getOmega() {
        return omega;
    }

    /**
     * @return The x component of the linear acceleration of the body
     */
    public double getAx() {
        return ax;
    }

    /**
     * @return The y component of the linear acceleration of the body
     */
    public double getAy() {
        return ay;
    }

    /**
     * @return The angular acceleration of the body
     */
    public double getAlpha() {
        return alpha;
    }
}
