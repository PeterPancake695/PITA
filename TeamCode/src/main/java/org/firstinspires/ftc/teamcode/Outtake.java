package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Outtake {
    FtcDashboard dashboard;

    PID3 pidIntake;
    public static double kP = 0.05, kI = 0, kD = 0;
    public static int targetSliderFull = 1600, targetSliderMiddle = 800, targetSliderRetracted = 0;
    public static double positionArmLeft = 0, positionArmRight = 0.6;
    public static double positionWristLeft = 0, positionWristRight = 0.6;
    public static double positionClawOpen = 0.1, positionClawClosed = 0;

    DcMotorEx motorSliderVertical;
    Servo servoArm, servoWrist, servoClaw;

    public enum sliderPos {
        FULL,
        MIDDLE
    }

    public enum sliderVertical {
        RETRACTED,
        EXTENDED
    }

    public enum arm {
        LEFT,
        RIGHT
    }

    public enum claw {
        OPEN,
        CLOSED
    }

    public enum wrist {
        LEFT,
        RIGHT
    }
    public static sliderPos caseSliderVerticalPos;

    public static sliderVertical caseSliderVertical;
    public static arm caseArm;
    public static claw caseClaw;
    public static wrist caseWrist;

    void hardware(HardwareMap hwmap) {
        motorSliderVertical = hwmap.get(DcMotorEx.class, "motorslidervertical");

        servoArm = hwmap.get(Servo.class, "servoarmouttake");
        servoWrist = hwmap.get(Servo.class, "servowristouttake");
        servoClaw = hwmap.get(Servo.class, "servoclawouttake");

        motorSliderVertical.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorSliderVertical.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSliderVertical.setDirection(DcMotor.Direction.REVERSE);
    }
    Outtake(HardwareMap hwmap) {
        dashboard = FtcDashboard.getInstance();

        pidIntake = new PID3(kP, kI, kD, 0);
        pidIntake.setOutputRange(-1.0, 1.0);
        pidIntake.setIntegralLimit(0.25);
        hardware(hwmap);
    }
    void runSlider() {
        int currentPosition = motorSliderVertical.getCurrentPosition();
        double power = 0;
        switch (caseSliderVertical) {
            case RETRACTED:
                power = pidIntake.update(targetSliderRetracted, currentPosition);
                break;
            case EXTENDED:
                switch (caseSliderVerticalPos) {
                    case FULL:
                        power = pidIntake.update(targetSliderFull, currentPosition);
                        break;
                    case MIDDLE:
                        power = pidIntake.update(targetSliderMiddle, currentPosition);
                        break;
                }
                break;
        }
        motorSliderVertical.setPower(power);
    }

    void runArm() {
        switch (caseArm) {
            case RIGHT:
                servoArm.setPosition(positionArmLeft);
                break;
            case LEFT:
                servoArm.setPosition(positionArmRight);
                break;
        }
    }

    void runWrist() {
        switch (caseWrist) {
            case LEFT:
                servoWrist.setPosition(positionWristLeft);
                break;
            case RIGHT:
                servoWrist.setPosition(positionWristRight);
                break;
        }
    }

    void runClaw() {
        switch (caseClaw) {
            case OPEN:
                servoClaw.setPosition(positionClawOpen);
                break;
            case CLOSED:
                servoClaw.setPosition(positionClawClosed);
                break;
        }
    }
}
