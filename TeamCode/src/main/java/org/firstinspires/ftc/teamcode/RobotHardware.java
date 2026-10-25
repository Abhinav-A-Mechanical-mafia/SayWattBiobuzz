
package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class RobotHardware {

    // Declare motors
    public DcMotor frontLeftDrive;
    public DcMotor backLeftDrive;
    public DcMotor frontRightDrive;
    public DcMotor backRightDrive;

    // Declare servos
    public CRServo servoLeft;
    public CRServo servoRight;
    public CRServo intakeServoFirst;

    // Initialize everything
    public void init(HardwareMap hardwareMap) {

        // Motors
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        // Servos
        servoLeft = hardwareMap.get(CRServo.class, "servo_left");
        servoRight = hardwareMap.get(CRServo.class, "servo_right");
        intakeServoFirst = hardwareMap.get(CRServo.class, "intake_first");

        // Motor directions
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontRightDrive.setDirection(DcMotor.Direction.FORWARD);
        backRightDrive.setDirection(DcMotor.Direction.FORWARD);
    }
}
