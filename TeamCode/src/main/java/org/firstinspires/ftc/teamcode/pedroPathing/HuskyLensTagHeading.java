package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.PathPoint;

/** Supplies a heading goal to Pedro while Pedro follows the original XY path. */
final class HuskyLensTagHeading implements HeadingInterpolator {
    private static final long LOSS_DELAY_NS = 250_000_000;
    private static final long SWEEP_STEP_NS = 500_000_000;
    private static final double SWEEP_ANGLE = Math.toRadians(15);
    private static final double MAX_SEARCH_TURN = Math.toRadians(35);

    private final HuskyLensLocalizer vision;
    private final HeadingInterpolator originalHeading;
    private long lostSince;
    private double lastAim = Double.NaN;
    private double searchStart, searchCenter;

    HuskyLensTagHeading(HuskyLensLocalizer vision, HeadingInterpolator originalHeading) {
        this.vision = vision;
        this.originalHeading = originalHeading;
    }

    @Override
    public double interpolate(PathPoint point) {
        long now = System.nanoTime();
        if (!vision.canAim(now)) {
            return originalHeading.interpolate(point);
        }
        double heading = vision.getPose().getHeading();
        double visibleHeading = vision.getTargetHeading();
        if (Double.isFinite(visibleHeading)) {
            lostSince = 0;
            lastAim = limitTurn(heading, visibleHeading, MAX_SEARCH_TURN);
            return lastAim;
        }

        if (lostSince == 0) {
            lostSince = now;
            searchStart = heading;
            searchCenter = limitTurn(searchStart, vision.searchHeading(), MAX_SEARCH_TURN - SWEEP_ANGLE);
        }
        long elapsed = now - lostSince;
        if (elapsed < LOSS_DELAY_NS) {
            return Double.isFinite(lastAim) ? lastAim : originalHeading.interpolate(point);
        }
        long sweep = elapsed - LOSS_DELAY_NS;
        if (sweep < SWEEP_STEP_NS) {
            return searchCenter;
        }
        if (sweep < 2 * SWEEP_STEP_NS) {
            return searchCenter + SWEEP_ANGLE;
        }
        if (sweep < 3 * SWEEP_STEP_NS) {
            return searchCenter - SWEEP_ANGLE;
        }
        return originalHeading.interpolate(point);
    }

    private static double limitTurn(double from, double to, double limit) {
        double turn = HuskyLensPoseSolver.angleDifference(to, from);
        return from + Math.max(-limit, Math.min(limit, turn));
    }
}
