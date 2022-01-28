package frckit.physics.drivetrain.follower;

import edu.wpi.first.math.geometry.Pose2d;
import frckit.physics.state.RigidBodyState2d;

/**
 * Interface for drivetrain trajectory followers.  Followers use the positional error, target state, and previous state
 * to produce a target state for the drivetrain to follow.  This state can then be converted into a command for the
 * individual wheels on the drivetrain using an appropriate dynamics class.
 */
public interface DrivetrainFeedback {
    RigidBodyState2d calculate(double dt, Pose2d error, RigidBodyState2d targetState, RigidBodyState2d lastUpdateState);
}
