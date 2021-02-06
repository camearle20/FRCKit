package frckit.vision;

import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frckit.vision.corner.Corner;
import frckit.vision.corner.CornerPair;

import java.util.Optional;
import java.util.function.Supplier;

public class DualCornerVisionKinematics {
    private Supplier<Optional<CornerPair>> cornerPairSupplier;

    private double vpw = 2.0 * Math.tan(Math.toRadians(59.6 / 2.0));
    private double vph = 2.0 * Math.tan(Math.toRadians(49.7 / 2.0));
    private Rotation2d horizontalPlaneToLens;
    private double lensHeight;
    private double goalHeight;

    public DualCornerVisionKinematics(
            Supplier<Optional<CornerPair>> cornerPairSupplier,
            Rotation2d horizontalPlaneToLens,
            double lensHeight,
            double goalHeight
    ) {
        this.horizontalPlaneToLens = horizontalPlaneToLens;
        this.lensHeight = lensHeight;
        this.goalHeight = goalHeight;
    }

    public void setVpw(double vpw) {
        this.vpw = vpw;
    }

    public void setVph(double vph) {
        this.vph = vph;
    }

    public void setHorizontalPlaneToLens(Rotation2d horizontalPlaneToLens) {
        this.horizontalPlaneToLens = horizontalPlaneToLens;
    }

    public void setLensHeight(double lensHeight) {
        this.lensHeight = lensHeight;
    }

    public void setGoalHeight(double goalHeight) {
        this.goalHeight = goalHeight;
    }

    private Optional<Translation2d> solveCameraToVisionTargetTranslation(Corner corner, double slope) {
        double yPixels = corner.getX();
        double zPixels = corner.getY();

        //Robot frame of reference
        double nY = -((yPixels - 160.0) / 160.0);
        double nZ = -((zPixels - 120.0) / 120.0);

        Translation2d xzPlaneTranslation = new Translation2d(1.0, vph / 2.0 * nZ).rotateBy(horizontalPlaneToLens);
        double x = xzPlaneTranslation.getX();
        double y = vpw / 2.0 * nY;
        double z = xzPlaneTranslation.getY();

        double differentialHeight = lensHeight - goalHeight;
        if ((z < 0.0) == (differentialHeight > 0.0)) {
            double scaling = differentialHeight / -z;
            double distance = Math.hypot(x, y) * scaling;
            Rotation2d angle = new Rotation2d(x, y);
            return Optional.of(new Translation2d(distance * angle.getCos(), distance * angle.getSin()));
        }
        return Optional.empty();
    }

    public Optional<Translation2d> forwardKinematics() {
        Optional<CornerPair> cornersOpt = cornerPairSupplier.get();
        if (cornersOpt.isEmpty()) {
            return Optional.empty(); //There are no corners, so no kinematics can be done
        }

        CornerPair corners = cornersOpt.get();

        double slope = 1.0;
        if (Math.abs(corners.getRight().getX() - corners.getLeft().getX()) > 1e-12) {
            slope = (corners.getRight().getY() - corners.getLeft().getY()) /
                    (corners.getRight().getX() - corners.getLeft().getX());
        }

        Optional<Translation2d> cameraToLeftVisionTargetOpt = solveCameraToVisionTargetTranslation(corners.getLeft(), slope);
        Optional<Translation2d> cameraToRightVisionTargetOpt = solveCameraToVisionTargetTranslation(corners.getRight(), slope);

        if (cameraToLeftVisionTargetOpt.isEmpty() || cameraToRightVisionTargetOpt.isEmpty()) {
            return Optional.empty();
        }

        Translation2d cameraToLeftVisionTarget = cameraToLeftVisionTargetOpt.get();
        Translation2d cameraToRightVisionTarget = cameraToRightVisionTargetOpt.get();

        //Interpolate to find center of goal
        Translation2d cameraToVisionTarget = new Translation2d(
                0.5 * (cameraToRightVisionTarget.getX() - cameraToLeftVisionTarget.getX()) + cameraToLeftVisionTarget.getX(),
                0.5 * (cameraToRightVisionTarget.getY() - cameraToLeftVisionTarget.getY()) + cameraToLeftVisionTarget.getY()
        );

        return Optional.of(cameraToVisionTarget);
    }
}
