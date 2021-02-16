package frckit.physics.drivetrain.follower;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import frckit.physics.state.RigidBodyState2d;

/**
 * Implements "no feedback", which simply outputs the target (trajectory) state as the updated state.  This is useful
 * for testing feedforward parameters.
 */
public class NoFeedback implements DrivetrainFeedback {
    @Override
    public RigidBodyState2d calculate(double dt, Pose2d error, RigidBodyState2d targetState, RigidBodyState2d lastUpdateState) {
        return targetState;
    }
}
