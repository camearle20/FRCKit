package frckit.physics.drivetrain.follower;

import edu.wpi.first.math.geometry.Pose2d;
import frckit.physics.state.RigidBodyState2d;
import frckit.util.Epsilon;

/**
 * Implements nonlinear feedback for a non-holonomic drivetrain (a differential drivetrain is an example of this).
 * This is called the "RAMSETE" algorithm in WPILib.
 */
public class NonholonomicNonlinearFeedback implements DrivetrainFeedback {
    private final double kBeta;
    private final double kZeta;

    public NonholonomicNonlinearFeedback(double kBeta, double kZeta) {
        this.kBeta = kBeta;
        this.kZeta = kZeta;
    }


    public NonholonomicNonlinearFeedback() {
        this(2.0, 0.7);
    }

    @Override
    public RigidBodyState2d calculate(double dt, Pose2d error, RigidBodyState2d targetState, RigidBodyState2d lastUpdateState) {
        //Unpack state
        double v = targetState.getVx();
        double omega = targetState.getOmega();
        double angleError = error.getRotation().getRadians();
        double sinc = Epsilon.equals(angleError, 0.0, 1e-9) ? 1.0 : error.getRotation().getSin() / angleError; //sin(theta) / theta = sinc(theta)

        //Ramsete
        double k = 2.0 * kZeta * Math.sqrt(kBeta * v * v + omega * omega); //Gain parameter

        double adjustedV = v * error.getRotation().getCos() + k * error.getTranslation().getX();
        double adjustedOmega = omega + k * angleError + v * kBeta * sinc * error.getTranslation().getY();

        double a = Epsilon.equals(dt, 0.0) ? 0.0 : (adjustedV - lastUpdateState.getVx()) / dt;
        double alpha = Epsilon.equals(dt, 0.0) ? 0.0 : (adjustedOmega - lastUpdateState.getOmega()) / dt;

        //Non-holonomic does not have any velocity or acceleration in the y direction (since this would make it holonomic!)
        return new RigidBodyState2d(adjustedV, 0.0, adjustedOmega, a, 0.0, alpha);
    }
}
