package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.HuskyLensTagMath;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/** Fits tag centers on the field to rays from the calibrated HuskyLens. */
final class HuskyLensPoseSolver {
    private static final double MIN_BASELINE_INCHES = 3;
    private static final double MAX_RANGE_INCHES = 120;
    private static final double MAX_POSITION_CORRECTION = 18;
    private static final double MAX_HEADING_CORRECTION = Math.toRadians(40);
    private static final double MAX_SINGLE_TAG_CORRECTION = 10;

    static final class Tag {
        final double x, y, height;

        Tag(double x, double y, double height) {
            this.x = x;
            this.y = y;
            this.height = height;
        }
    }

    static final class Observation {
        final Tag tag;
        final double forward, left, tolerance, weight;
        final boolean widthBased;

        Observation(Tag tag, double forward, double left, boolean widthBased) {
            this.tag = tag;
            this.forward = forward;
            this.left = left;
            this.widthBased = widthBased;
            double range = Math.hypot(forward, left);
            // Apparent width varies with the tag's angle, so give it less weight.
            tolerance = widthBased ? Math.max(2.5, 0.10 * range) : Math.max(1.75, 0.04 * range);
            weight = 1 / (tolerance * tolerance);
        }

        double residual(Pose camera) {
            double cos = Math.cos(camera.getHeading());
            double sin = Math.sin(camera.getHeading());
            return Math.hypot(camera.getX() + forward * cos - left * sin - tag.x,
                    camera.getY() + forward * sin + left * cos - tag.y) / tolerance;
        }
    }

    static final class Fix {
        final Pose pose;
        final int tagCount, widthBasedCount;
        final boolean hasHeading;
        final double error;

        Fix(Pose pose, int tagCount, int widthBasedCount, boolean hasHeading, double error) {
            this.pose = pose;
            this.tagCount = tagCount;
            this.widthBasedCount = widthBasedCount;
            this.hasHeading = hasHeading;
            this.error = error;
        }
    }

    private final double lensHeight, cameraForward, cameraLeft, cameraYaw;
    private final double sinPitch, cosPitch, fx, fy, cx, cy, tagWidthInches;

    HuskyLensPoseSolver(double height, double forward, double left, double yaw, double pitch,
                       double fx, double fy, double cx, double cy, double tagWidthInches) {
        lensHeight = height;
        cameraForward = forward;
        cameraLeft = left;
        cameraYaw = yaw;
        sinPitch = Math.sin(pitch);
        cosPitch = Math.cos(pitch);
        this.fx = fx;
        this.fy = fy;
        this.cx = cx;
        this.cy = cy;
        this.tagWidthInches = tagWidthInches;
    }

    double bearing(double pixelX, double pixelY) {
        double up = (cy - pixelY) / fy;
        return Math.atan2(-HuskyLensTagMath.horizontalSlope(pixelX, cx, fx), cosPitch - up * sinPitch);
    }

    double headingToTag(Pose robot, Tag tag) {
        double h = robot.getHeading();
        double cameraX = robot.getX() + cameraForward * Math.cos(h) - cameraLeft * Math.sin(h);
        double cameraY = robot.getY() + cameraForward * Math.sin(h) + cameraLeft * Math.cos(h);
        return Math.atan2(tag.y - cameraY, tag.x - cameraX) - cameraYaw;
    }

    Observation observe(Tag tag, double pixelX, double pixelY, double widthPixels) {
        double up = (cy - pixelY) / fy;
        double forwardSlope = cosPitch - up * sinPitch;
        double leftSlope = -HuskyLensTagMath.horizontalSlope(pixelX, cx, fx);
        double verticalSlope = sinPitch + up * cosPitch;
        double heightDifference = tag.height - lensHeight;
        if (lensHeight > 0 && tag.height > 0 && Math.abs(heightDifference) >= 1
                && Math.abs(verticalSlope) >= 0.035) {
            // Measured heights let us intersect the ray with the tag's horizontal plane.
            Observation heightFix = observation(tag, heightDifference / verticalSlope,
                    forwardSlope, leftSlope, false);
            if (heightFix != null) {
                return heightFix;
            }
        }
        // Width gives an approximate depth even when the tag is level with the lens.
        double depth = HuskyLensTagMath.depthFromWidth(tagWidthInches, fx, widthPixels);
        return observation(tag, depth, forwardSlope, leftSlope, true);
    }

    private static Observation observation(Tag tag, double depth, double forwardSlope,
                                           double leftSlope, boolean widthBased) {
        double forward = depth * forwardSlope;
        double left = depth * leftSlope;
        if (!(depth > 0 && forward > 0 && Math.hypot(forward, left) <= MAX_RANGE_INCHES)) {
            return null;
        }
        return new Observation(tag, forward, left, widthBased);
    }

    Fix estimate(List<Observation> observations, Pose predicted, double headingBeforeFirstFix) {
        double fieldHeading = predicted == null ? headingBeforeFirstFix : predicted.getHeading();
        Fix best = null;
        // Each separated pair proposes a pose. Refit using all observations that agree.
        for (int i = 0; i < observations.size(); i++) {
            Observation a = observations.get(i);
            for (int j = i + 1; j < observations.size(); j++) {
                Observation b = observations.get(j);
                double fieldBaseline = Math.hypot(b.tag.x - a.tag.x, b.tag.y - a.tag.y);
                double seenBaseline = Math.hypot(b.forward - a.forward, b.left - a.left);
                if (fieldBaseline < MIN_BASELINE_INCHES || seenBaseline < 2
                        || Math.abs(fieldBaseline - seenBaseline) > 0.6 + 0.35 * fieldBaseline) {
                    continue;
                }
                Pose seed = fitCamera(Arrays.asList(a, b), 0, true);
                Fix candidate = evaluate(observations, seed, predicted, fieldHeading, true);
                best = better(best, candidate);
            }
        }
        if (best != null || !Double.isFinite(fieldHeading)) {
            return best;
        }

        // Field heading lets a single tag anchor x and y, including the first fix.
        double cameraHeading = fieldHeading + cameraYaw;
        for (Observation observation : observations) {
            Pose seed = cameraFromPoint(observation.forward, observation.left,
                    observation.tag.x, observation.tag.y, cameraHeading);
            best = better(best, evaluate(observations, seed, predicted, fieldHeading, false));
        }
        return best;
    }

    private Fix evaluate(List<Observation> observations, Pose seed, Pose predicted,
                         double fieldHeading, boolean fitHeading) {
        List<Observation> agreeing = new ArrayList<>();
        for (Observation observation : observations) {
            if (observation.residual(seed) <= 1) {
                agreeing.add(observation);
            }
        }
        int minimum = fitHeading ? Math.max(2, (observations.size() + 1) / 2) : 1;
        if (!fitHeading && predicted == null) {
            minimum = observations.size() / 2 + 1;
        }
        if (agreeing.size() < minimum) {
            return null;
        }

        Pose camera = fitCamera(agreeing, seed.getHeading(), fitHeading);
        double heading = camera.getHeading() - cameraYaw;
        Pose robot = new Pose(
                camera.getX() - cameraForward * Math.cos(heading) + cameraLeft * Math.sin(heading),
                camera.getY() - cameraForward * Math.sin(heading) - cameraLeft * Math.cos(heading),
                heading);
        if (!onField(robot.getX(), robot.getY())) {
            return null;
        }
        double error = 0;
        int inliers = 0, widthBasedCount = 0;
        for (Observation observation : agreeing) {
            double residual = observation.residual(camera);
            if (residual <= 1) {
                error += residual * residual;
                inliers++;
                if (observation.widthBased) {
                    widthBasedCount++;
                }
            }
        }
        if (inliers < minimum) {
            return null;
        }
        error /= inliers;
        if (Double.isFinite(fieldHeading)) {
            double turn = Math.abs(angleDifference(heading, fieldHeading));
            if (turn > MAX_HEADING_CORRECTION) {
                return null;
            }
            error += 0.3 * turn;
        }
        if (predicted != null) {
            double distance = Math.hypot(robot.getX() - predicted.getX(), robot.getY() - predicted.getY());
            if (distance > (fitHeading ? MAX_POSITION_CORRECTION : MAX_SINGLE_TAG_CORRECTION)) {
                return null;
            }
            error += 0.025 * distance;
        }
        return new Fix(robot, inliers, widthBasedCount, fitHeading, error);
    }

    private static Pose fitCamera(List<Observation> observations, double heading, boolean fitHeading) {
        double sum = 0, forward = 0, left = 0, x = 0, y = 0;
        for (Observation observation : observations) {
            double weight = observation.weight;
            sum += weight;
            forward += weight * observation.forward;
            left += weight * observation.left;
            x += weight * observation.tag.x;
            y += weight * observation.tag.y;
        }
        forward /= sum;
        left /= sum;
        x /= sum;
        y /= sum;
        if (fitHeading) {
            double dot = 0, cross = 0;
            for (Observation observation : observations) {
                double df = observation.forward - forward, dl = observation.left - left;
                double dx = observation.tag.x - x, dy = observation.tag.y - y;
                dot += observation.weight * (df * dx + dl * dy);
                cross += observation.weight * (df * dy - dl * dx);
            }
            heading = Math.atan2(cross, dot);
        }
        return cameraFromPoint(forward, left, x, y, heading);
    }

    private static Pose cameraFromPoint(double forward, double left, double x, double y, double heading) {
        return new Pose(x - forward * Math.cos(heading) + left * Math.sin(heading),
                y - forward * Math.sin(heading) - left * Math.cos(heading), heading);
    }

    private static Fix better(Fix best, Fix candidate) {
        if (candidate != null && (best == null || candidate.tagCount > best.tagCount
                || (candidate.tagCount == best.tagCount && candidate.error < best.error))) {
            return candidate;
        }
        return best;
    }

    static double angleDifference(double to, double from) {
        double difference = to - from;
        return Math.atan2(Math.sin(difference), Math.cos(difference));
    }

    static boolean onField(double x, double y) {
        return x >= 0 && x <= 144 && y >= 0 && y <= 144;
    }
}
