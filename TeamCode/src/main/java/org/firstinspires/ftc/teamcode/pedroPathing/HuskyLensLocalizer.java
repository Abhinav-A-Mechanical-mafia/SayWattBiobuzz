package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.ftc.localization.localizers.DriveEncoderLocalizer;
import com.pedropathing.geometry.Pose;
import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Vector;
import com.pedropathing.paths.HeadingInterpolator;
import com.qualcomm.hardware.dfrobot.HuskyLens;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.HuskyLensTagMath;

import java.util.ArrayList;
import java.util.HashSet;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.StringJoiner;

@Configurable
public class HuskyLensLocalizer implements Localizer {
    // Measure tag centers in Pedro field inches. Zero height uses range from tag width.
    public static boolean fixedLayoutMeasured = false;
    public static int tag1Id = 30, tag1BlockId = 0;
    public static double tag1X, tag1Y, tag1Height;
    public static int tag2Id = 31, tag2BlockId = 0;
    public static double tag2X, tag2Y, tag2Height;
    public static int tag3Id = 32, tag3BlockId = 0;
    public static double tag3X, tag3Y, tag3Height;
    // Use "printedId,x,y" or append ",height,learnedId". BIOBUZZ IDs 30 through 45 default to 1 through 16.
    public static String additionalFixedTags = "";

    // Level the camera side to side. Yaw is positive left and pitch is positive up.
    public static double cameraHeight;
    public static double cameraForwardInches, cameraLeftInches;
    public static double cameraYawDegrees, cameraPitchDegrees;
    public static double focalX = HuskyLensTagMath.DEFAULT_FOCAL_LENGTH_PIXELS;
    public static double focalY = HuskyLensTagMath.DEFAULT_FOCAL_LENGTH_PIXELS;
    public static double centerX = HuskyLensTagMath.DEFAULT_CENTER_X_PIXELS, centerY = 120;
    public static double tagWidthInches = HuskyLensTagMath.BIOBUZZ_TAG_WIDTH_INCHES;

    private static final int IMAGE_WIDTH = 320, IMAGE_HEIGHT = 240;
    private static final long POLL_INTERVAL_NS = 100_000_000;
    private static final long MAX_SAMPLE_AGE_NS = 300_000_000;

    private final Localizer encoders;
    private final Map<Integer, HuskyLensPoseSolver.Tag> tags = new LinkedHashMap<>();
    private final HuskyLensPoseSolver solver;
    private final HuskyLens camera;
    private final String layoutStatus;
    private Pose previousEncoderPose;
    private Pose pose = new Pose();
    private double totalHeading;
    private double headingBeforeFirstFix = Double.NaN;
    private boolean fieldPoseKnown;
    private boolean encodersHealthy = true;
    private long lastPoll, lastSample;
    private double targetHeading = Double.NaN;
    private int lastTargetId;
    private String reportedIds = "";
    private String cameraStatus = "Waiting for a camera frame";

    public HuskyLensLocalizer(HardwareMap hardwareMap) {
        this(hardwareMap, new DriveEncoderLocalizer(hardwareMap, Constants.localizerConstants));
    }

    HuskyLensLocalizer(HardwareMap hardwareMap, Localizer encoders) {
        this.encoders = encoders;
        previousEncoderPose = encoders.getPose();
        solver = cameraModel();
        layoutStatus = readTagMap();
        camera = configureCamera(hardwareMap);
    }

    private HuskyLens configureCamera(HardwareMap hardwareMap) {
        try {
            HuskyLens device = hardwareMap.tryGet(HuskyLens.class, "huskylens");
            if (device != null && device.knock()) {
                device.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
                return device;
            }
            cameraStatus = "No response from the configured huskylens";
        } catch (RuntimeException error) {
            cameraStatus = "HuskyLens initialization failed: " + error.getClass().getSimpleName();
        }
        return null;
    }

    private static HuskyLensPoseSolver cameraModel() {
        if (!Double.isFinite(cameraForwardInches) || !Double.isFinite(cameraLeftInches)
                || !Double.isFinite(cameraYawDegrees) || !(Math.abs(cameraPitchDegrees) < 90)
                || !(cameraHeight >= 0 && Double.isFinite(cameraHeight))
                || !(focalX > 0 && Double.isFinite(focalX) && focalY > 0 && Double.isFinite(focalY))
                || !(tagWidthInches > 0 && Double.isFinite(tagWidthInches))
                || !(centerX >= 0 && centerX < IMAGE_WIDTH && centerY >= 0 && centerY < IMAGE_HEIGHT)) {
            return null;
        }
        return new HuskyLensPoseSolver(cameraHeight, cameraForwardInches, cameraLeftInches,
                Math.toRadians(cameraYawDegrees), Math.toRadians(cameraPitchDegrees),
                focalX, focalY, centerX, centerY, tagWidthInches);
    }

    private String readTagMap() {
        if (solver == null || !fixedLayoutMeasured) {
            return "Enter the camera and surveyed tag measurements in Panels, then re-INIT";
        }
        Set<Integer> printedIds = new HashSet<>();
        try {
            if (tag1X != 0 || tag1Y != 0 || tag1Height != 0) {
                addTag(printedIds, tag1Id, tag1BlockId, tag1X, tag1Y, tag1Height);
            }
            if (tag2X != 0 || tag2Y != 0 || tag2Height != 0) {
                addTag(printedIds, tag2Id, tag2BlockId, tag2X, tag2Y, tag2Height);
            }
            if (tag3X != 0 || tag3Y != 0 || tag3Height != 0) {
                addTag(printedIds, tag3Id, tag3BlockId, tag3X, tag3Y, tag3Height);
            }
            for (String row : additionalFixedTags.split(";")) {
                if (row.trim().isEmpty()) {
                    continue;
                }
                String[] fields = row.trim().split(",", -1);
                if (fields.length < 3 || fields.length > 5) {
                    throw new IllegalArgumentException("Use printedId,x,y with optional height and learned ID");
                }
                double height = fields.length >= 4 ? Double.parseDouble(fields[3].trim()) : 0;
                int learned = fields.length == 5 ? Integer.parseInt(fields[4].trim()) : 0;
                addTag(printedIds, Integer.parseInt(fields[0].trim()), learned,
                        Double.parseDouble(fields[1].trim()), Double.parseDouble(fields[2].trim()),
                        height);
            }
        } catch (IllegalArgumentException invalidTag) {
            tags.clear();
            return "Tag map: " + invalidTag.getMessage();
        }
        return tags.isEmpty() ? "Add surveyed tag centers, then re-INIT" : "";
    }

    private void addTag(Set<Integer> printedIds, int printed, int learned, double x, double y, double height) {
        if (learned == 0 && printed >= 30 && printed <= 45) {
            learned = printed - 29;
        }
        if (learned <= 0 || learned > 65535) {
            throw new IllegalArgumentException("Enter the reported learned ID for tag " + printed);
        }
        if (printed < 0 || printed > 586 || !HuskyLensPoseSolver.onField(x, y)
                || !(height >= 0 && Double.isFinite(height))) {
            throw new IllegalArgumentException("Invalid tag center or printed ID: " + printed);
        }
        if (!printedIds.add(printed) || tags.containsKey(learned)) {
            throw new IllegalArgumentException("Duplicate printed or learned ID for tag " + printed);
        }
        tags.put(learned, new HuskyLensPoseSolver.Tag(x, y, height));
    }

    @Override
    public void update() {
        updateOdometry();
        long now = System.nanoTime();
        if (camera == null || (lastPoll != 0 && now - lastPoll < POLL_INTERVAL_NS)) {
            return;
        }
        lastPoll = now;
        HuskyLens.Block[] blocks;
        try {
            blocks = camera.blocks();
        } catch (RuntimeException error) {
            lastSample = 0;
            cameraStatus = "HuskyLens read failed: " + error.getClass().getSimpleName();
            return;
        }
        long finished = System.nanoTime();
        if (finished - now > MAX_SAMPLE_AGE_NS) {
            lastSample = 0;
            cameraStatus = "HuskyLens frame timed out";
            return;
        }
        readFrame(blocks, finished);
    }

    private void updateOdometry() {
        encoders.update();
        Pose current = encoders.getPose();
        encodersHealthy = finite(current);
        if (!encodersHealthy) {
            return;
        }
        double turn = HuskyLensPoseSolver.angleDifference(current.getHeading(), previousEncoderPose.getHeading());
        if (fieldPoseKnown) {
            double dx = current.getX() - previousEncoderPose.getX();
            double dy = current.getY() - previousEncoderPose.getY();
            double offset = pose.getHeading() - previousEncoderPose.getHeading();
            pose = new Pose(pose.getX() + dx * Math.cos(offset) - dy * Math.sin(offset),
                    pose.getY() + dx * Math.sin(offset) + dy * Math.cos(offset), pose.getHeading() + turn);
        } else if (Double.isFinite(headingBeforeFirstFix)) {
            headingBeforeFirstFix += turn;
        }
        totalHeading += turn;
        previousEncoderPose = current;
    }

    private void readFrame(HuskyLens.Block[] blocks, long capturedAt) {
        lastSample = capturedAt;
        StringJoiner ids = new StringJoiner(", ");
        Map<Integer, HuskyLens.Block> unique = new LinkedHashMap<>();
        Set<Integer> duplicates = new HashSet<>();
        HuskyLens.Block target = null;
        double bestScore = 0;
        for (HuskyLens.Block block : blocks) {
            ids.add(Integer.toString(block.id));
            if (!inImage(block)) {
                continue;
            }
            if (block.id > 0 && unique.putIfAbsent(block.id, block) != null) {
                duplicates.add(block.id);
            }
            double score = targetScore(block);
            if (score > bestScore) {
                target = block;
                bestScore = score;
            }
        }
        reportedIds = ids.toString();
        if (solver == null) {
            cameraStatus = "Camera ready";
            return;
        }

        List<HuskyLensPoseSolver.Observation> observations = new ArrayList<>();
        for (Map.Entry<Integer, HuskyLens.Block> entry : unique.entrySet()) {
            HuskyLensPoseSolver.Tag tag = tags.get(entry.getKey());
            HuskyLens.Block block = entry.getValue();
            if (tag != null && !duplicates.contains(entry.getKey()) && fullyInFrame(block)) {
                HuskyLensPoseSolver.Observation observation = solver.observe(tag, block.x, block.y, block.width);
                if (observation != null) {
                    observations.add(observation);
                }
            }
        }
        HuskyLensPoseSolver.Fix fix = solver.estimate(observations, fieldPoseKnown ? pose : null,
                headingBeforeFirstFix);
        if (fix == null) {
            if (fieldPoseKnown) {
                cameraStatus = "Drive encoders (" + observations.size() + " usable tag centers)";
            } else if (Double.isFinite(headingBeforeFirstFix)) {
                cameraStatus = "Waiting for a usable mapped tag";
            } else {
                cameraStatus = "Enter a start heading or show two mapped tags";
            }
        } else {
            applyFix(fix);
            String tagCount = fix.tagCount == 1 ? "1 tag" : fix.tagCount + " tags";
            String rangeMethod = fix.widthBasedCount > 0 ? ", width range" : "";
            cameraStatus = (fix.hasHeading ? "Vision position + heading" : "Vision position + encoder heading")
                    + " (" + tagCount + rangeMethod + ")";
        }

        targetHeading = target != null && fieldPoseKnown
                ? pose.getHeading() + solver.bearing(target.x, target.y) : Double.NaN;
        if (target != null) {
            lastTargetId = target.id;
        }
    }

    private void applyFix(HuskyLensPoseSolver.Fix fix) {
        if (!fieldPoseKnown) {
            pose = fix.pose;
            fieldPoseKnown = true;
            headingBeforeFirstFix = Double.NaN;
            return;
        }
        double xyGain = fix.hasHeading ? 0.4 : 0.25;
        double headingStep = 0.35 * HuskyLensPoseSolver.angleDifference(fix.pose.getHeading(), pose.getHeading());
        pose = new Pose(pose.getX() + xyGain * (fix.pose.getX() - pose.getX()),
                pose.getY() + xyGain * (fix.pose.getY() - pose.getY()), pose.getHeading() + headingStep);
    }

    private double targetScore(HuskyLens.Block block) {
        double offCenter = (block.x - centerX) / (IMAGE_WIDTH / 2.0);
        double continuity = block.id > 0 && block.id == lastTargetId ? 1.35 : 1;
        double mapped = tags.containsKey(block.id) ? 1.5 : 1;
        return continuity * mapped * block.width * block.height / (1 + offCenter * offCenter);
    }

    private static boolean inImage(HuskyLens.Block block) {
        return block.width >= 5 && block.height >= 5
                && block.width <= IMAGE_WIDTH && block.height <= IMAGE_HEIGHT
                && block.x >= 0 && block.x < IMAGE_WIDTH && block.y >= 0 && block.y < IMAGE_HEIGHT;
    }

    private static boolean fullyInFrame(HuskyLens.Block block) {
        return block.x - block.width / 2.0 > 1 && block.x + block.width / 2.0 < IMAGE_WIDTH - 1
                && block.y - block.height / 2.0 > 1 && block.y + block.height / 2.0 < IMAGE_HEIGHT - 1;
    }

    boolean canAim(long now) {
        return !isNAN() && solver != null && lastSample != 0 && now - lastSample <= MAX_SAMPLE_AGE_NS;
    }

    double getTargetHeading() {
        return targetHeading;
    }

    double searchHeading() {
        double heading = pose.getHeading();
        double bestHeading = heading;
        double bestScore = Double.POSITIVE_INFINITY;
        for (Map.Entry<Integer, HuskyLensPoseSolver.Tag> entry : tags.entrySet()) {
            HuskyLensPoseSolver.Tag tag = entry.getValue();
            double distance = Math.hypot(tag.x - pose.getX(), tag.y - pose.getY());
            if (distance < 3 || distance > 120) {
                continue;
            }
            double bearing = solver.headingToTag(pose, tag);
            double turn = Math.abs(HuskyLensPoseSolver.angleDifference(bearing, heading));
            double score = turn + 0.004 * distance;
            if (entry.getKey() == lastTargetId) {
                score -= Math.toRadians(10);
            }
            if (score < bestScore) {
                bestHeading = bearing;
                bestScore = score;
            }
        }
        return bestHeading;
    }

    /** Use on a path segment where the chassis is free to turn the fixed camera. */
    public HeadingInterpolator faceTags(HeadingInterpolator pathHeading) {
        return new HuskyLensTagHeading(this, pathHeading);
    }

    public String getStatus() {
        if (!encodersHealthy) {
            return "Invalid drive encoder pose";
        }
        return layoutStatus.isEmpty() ? cameraStatus : cameraStatus + ". " + layoutStatus;
    }

    public String getReportedIds() {
        return reportedIds.isEmpty() ? "None" : reportedIds;
    }

    public String getTargetStatus() {
        if (!canAim(System.nanoTime()) || !Double.isFinite(targetHeading)) {
            return "No visible target";
        }
        return lastTargetId == 0 ? "Unlearned tag" : "Learned ID " + lastTargetId;
    }

    @Override public Pose getPose() { return pose; }

    @Override
    public Pose getVelocity() {
        Pose velocity = encoders.getVelocity();
        if (isNAN() || !finite(velocity)) {
            return new Pose();
        }
        double offset = pose.getHeading() - previousEncoderPose.getHeading();
        return new Pose(velocity.getX() * Math.cos(offset) - velocity.getY() * Math.sin(offset),
                velocity.getX() * Math.sin(offset) + velocity.getY() * Math.cos(offset), velocity.getHeading());
    }

    @Override public Vector getVelocityVector() { return getVelocity().getAsVector(); }
    @Override public double getTotalHeading() { return totalHeading; }
    @Override public double getForwardMultiplier() { return encoders.getForwardMultiplier(); }
    @Override public double getLateralMultiplier() { return encoders.getLateralMultiplier(); }
    @Override public double getTurningMultiplier() { return encoders.getTurningMultiplier(); }
    @Override public void resetIMU() throws InterruptedException { encoders.resetIMU(); }
    @Override public double getIMUHeading() { return encoders.getIMUHeading(); }
    @Override public boolean isNAN() { return !fieldPoseKnown || !encodersHealthy; }

    @Override public void setStartPose(Pose startPose) { setPose(startPose); }

    /** Seeds Pedro's field heading in radians when the camera will provide the initial position. */
    public void setStartingFieldHeading(double heading) {
        if (!Double.isFinite(heading)) {
            throw new IllegalArgumentException("The starting field heading must be finite");
        }
        headingBeforeFirstFix = heading;
        previousEncoderPose = encoders.getPose();
        totalHeading = 0;
        fieldPoseKnown = false;
        targetHeading = Double.NaN;
    }

    @Override
    public void setPose(Pose newPose) {
        if (!finite(newPose)) {
            throw new IllegalArgumentException("The starting field pose must be finite");
        }
        pose = newPose.copy();
        previousEncoderPose = encoders.getPose();
        totalHeading = 0;
        fieldPoseKnown = true;
        headingBeforeFirstFix = Double.NaN;
        targetHeading = Double.NaN;
    }

    private static boolean finite(Pose candidate) {
        return Double.isFinite(candidate.getX()) && Double.isFinite(candidate.getY())
                && Double.isFinite(candidate.getHeading());
    }
}
