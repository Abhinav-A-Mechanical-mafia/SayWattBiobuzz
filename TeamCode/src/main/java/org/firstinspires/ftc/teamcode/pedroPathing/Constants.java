package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.Mecanum;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.DriveEncoderConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

@Configurable
public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            //.translationalPIDFCoefficients(new PIDFCoefficients(0.06, 0, 0.0001, 0.025)) //add updated PIDF values after tuning
            .mass(10);//update mass in KGs
    @IgnoreConfigurable
    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("front_right_drive")
            .rightRearMotorName("back_right_drive")
            .leftRearMotorName("back_left_drive")
            .leftFrontMotorName("front_left_drive")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD);
    @IgnoreConfigurable
    public static DriveEncoderConstants localizerConstants = new DriveEncoderConstants()
            .rightFrontMotorName("front_right_drive")
            .rightRearMotorName("back_right_drive")
            .leftRearMotorName("back_left_drive")
            .leftFrontMotorName("front_left_drive")
            .robotWidth(17) /*Add width in inches*/
            .robotLength(16.75) /*Add length in inches*/
            .turnTicksToInches(-0.03) /*run the turn tuner auto and rotate the robot once counterclockwise and replace 99 with the number that it results in */
            .strafeTicksToInches(0.0383) /*run the lateral tuner auto and push the robot left 48 inches and replace 99 with the number that it results in */
            .forwardTicksToInches(0.0398) /*run the forward tuner auto and push the robot forward 48 inches and replace 99 with the number that it results in */
            .leftFrontEncoderDirection(Encoder.REVERSE)
            .leftRearEncoderDirection(Encoder.REVERSE)
            .rightFrontEncoderDirection(Encoder.FORWARD)
            .rightRearEncoderDirection(Encoder.FORWARD);

    @IgnoreConfigurable
    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        Mecanum drivetrain = new Mecanum(hardwareMap, driveConstants);
        for (DcMotor motor : drivetrain.getMotors()) {
            motor.setPower(0);
            motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
        HuskyLensLocalizer vision = new HuskyLensLocalizer(hardwareMap);
        Follower follower = new Follower(followerConstants, vision, drivetrain, pathConstraints) {
            @Override
            public void updateDrivetrain() {
                if (vision.isNAN()) {
                    breakFollowing();
                    return;
                }
                super.updateDrivetrain();
            }
        };
        follower.setMaxPower(0.20);
        return follower;
    }

    // The original tuning menu includes drive-encoder calibration and diagnostics.
    static Follower createDriveEncoderFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .driveEncoderLocalizer(localizerConstants)
                /* other builder steps */
                .build();
    }
}
