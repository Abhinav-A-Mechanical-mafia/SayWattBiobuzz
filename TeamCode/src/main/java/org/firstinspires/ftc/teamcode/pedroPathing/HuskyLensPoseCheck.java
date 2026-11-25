package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

@Configurable
@TeleOp(name = "HuskyLens Pose Check", group = "Pedro Pathing")
public class HuskyLensPoseCheck extends LinearOpMode {
    public static boolean useMeasuredStartPose = false;
    public static boolean useMeasuredStartHeading = false;
    public static double startXInches, startYInches, startHeadingDegrees;

    @Override
    public void runOpMode() {
        Follower follower = Constants.createFollower(hardwareMap);
        HuskyLensLocalizer vision = (HuskyLensLocalizer) follower.getPoseTracker().getLocalizer();
        if (useMeasuredStartPose && HuskyLensPoseSolver.onField(startXInches, startYInches)
                && Double.isFinite(startHeadingDegrees)) {
            follower.setStartingPose(new Pose(startXInches, startYInches, Math.toRadians(startHeadingDegrees)));
        } else if (useMeasuredStartHeading && Double.isFinite(startHeadingDegrees)) {
            vision.setStartingFieldHeading(Math.toRadians(startHeadingDegrees));
        }

        while (opModeInInit()) {
            follower.updatePose();
            reportPose(follower, vision);
            telemetry.addLine("Verify the field pose and learned tag IDs before running the short path.");
            telemetry.update();
            sleep(30);
        }

        ElapsedTime pathTimer = new ElapsedTime();
        boolean autonomous = startOnOpModeStart();
        if (autonomous && opModeIsActive()) {
            follower.updatePose();
            if (vision.isNAN()) {
                telemetry.addLine("Auto canceled. Get a vision fix or enter the measured start pose.");
                telemetry.update();
                return;
            }
            startShortPath(follower, vision);
            pathTimer.reset();
        }
        try {
            while (opModeIsActive()) {
                boolean stopPath = gamepad1.bWasPressed() || (follower.isBusy() && pathTimer.seconds() > 4);
                if (stopPath) {
                    follower.breakFollowing();
                }
                follower.update();
                if (!autonomous && gamepad1.aWasPressed() && !stopPath && !vision.isNAN() && !follower.isBusy()) {
                    pathTimer.reset();
                    startShortPath(follower, vision);
                }
                reportPose(follower, vision);
                telemetry.addLine(autonomous ? "Six-inch path with tag aiming. B: stop."
                        : "A: run a six-inch path with tag aiming. B: stop.");
                telemetry.update();
                if (autonomous && !follower.isBusy()) {
                    break;
                }
                sleep(20);
            }
        } finally {
            follower.breakFollowing();
        }
    }

    protected boolean startOnOpModeStart() {
        return false;
    }

    private void reportPose(Follower follower, HuskyLensLocalizer vision) {
        telemetry.addData("Localization", vision.getStatus());
        telemetry.addData("Reported tag IDs", vision.getReportedIds());
        telemetry.addData("Aim target", vision.getTargetStatus());
        if (!vision.isNAN()) {
            Pose current = follower.getPose();
            telemetry.addData("Pedro X/Y (in)", "%.1f / %.1f", current.getX(), current.getY());
            telemetry.addData("Heading (deg)", "%.1f", Math.toDegrees(current.getHeading()));
        }
    }

    private void startShortPath(Follower follower, HuskyLensLocalizer vision) {
        Pose from = follower.getPose();
        double heading = from.getHeading();
        double x = from.getX() + 6 * Math.cos(heading);
        double y = from.getY() + 6 * Math.sin(heading);
        if (x < 2 || x > 142 || y < 2 || y > 142) {
            telemetry.addLine("Test path would end outside the field.");
            return;
        }
        Path path = new Path(new BezierLine(from, new Pose(x, y, heading)));
        path.setHeadingInterpolation(vision.faceTags(HeadingInterpolator.constant(heading)));
        follower.followPath(path, false);
    }
}
