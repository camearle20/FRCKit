package frckit.physics;

import edu.wpi.first.wpilibj.geometry.Twist2d;
import frckit.util.GeomUtil;

/**
 * Represents the state of a rigid body, such as a drivetrain or other mechanism, in terms of its velocity and acceleration
 */
public class RigidBodyState {
    private final Twist2d velocity;
    private final Twist2d acceleration;

    public RigidBodyState() {
        velocity = GeomUtil.IDENTITY_TWIST;
        acceleration = GeomUtil.IDENTITY_TWIST;
    }

    public RigidBodyState(Twist2d velocity, Twist2d acceleration) {
        this.velocity = velocity;
        this.acceleration = acceleration;
    }

    public Twist2d getVelocity() {
        return velocity;
    }

    public Twist2d getAcceleration() {
        return acceleration;
    }
}
