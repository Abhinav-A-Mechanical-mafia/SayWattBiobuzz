package org.firstinspires.ftc.teamcode;

/** Pinhole projection for a tag detected by HuskyLens. */
public final class HuskyLensTagMath {
    public static final double BIOBUZZ_TAG_WIDTH_INCHES = 3.25;
    public static final double DEFAULT_FOCAL_LENGTH_PIXELS = 320.4;
    public static final double DEFAULT_CENTER_X_PIXELS = 160.0;

    private HuskyLensTagMath() {}

    /** Estimates depth along the optical axis from the apparent width of a square-on tag. */
    public static double depthFromWidth(double tagWidthInches, double focalPixels, double widthPixels) {
        return tagWidthInches * focalPixels / widthPixels;
    }

    /** The image ray's rightward distance per unit of optical depth. */
    public static double horizontalSlope(double pixelX, double centerX, double focalPixels) {
        return (pixelX - centerX) / focalPixels;
    }
}
