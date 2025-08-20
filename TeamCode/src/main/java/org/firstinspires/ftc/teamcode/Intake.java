package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

@Config
public class Intake {
    FtcDashboard dashboard;

    PID3 pidIntake;
    public static double kP = 0.05, kI = 0, kD = 0;
    public static int targetSliderExtended = 1600, targetSliderRetracted = 0, targetSliderTransfer = 200;
    public static double positionArmUp = 0.3, positionArmDown = 0.63, positionArmHovering = 0.5, positionArmTransfer = 0;
    public static double positionWristHorizontal = 0.02, positionWristVertical = 0.35;
    public static double positionClawOpen = 0.43, positionClawClosed = 0.69;


    DcMotorEx motorSliderHorizontal;
    Servo servoArm, servoWrist, servoClaw;

    public enum sliderHorizontal {
        RETRACTED,
        EXTENDED,
        TRANSFER
    }

    public enum arm {
        UP,
        DOWN,
        HOVERING,
        TRANSFER
    }

    public enum wrist {
        HORIZONTAL,
        VERTICAL,
    }

    public enum claw {
        OPEN,
        CLOSED
    }
    public static sliderHorizontal caseSliderHorizontal;
    public static arm caseArm;
    public static wrist caseWrist;
    public static  claw caseClaw;
    void hardware(HardwareMap hwmap) {
        motorSliderHorizontal = hwmap.get(DcMotorEx.class, "motorsliderhorizontal");
        servoArm = hwmap.get(Servo.class, "servoarmintake");
        servoWrist = hwmap.get(Servo.class, "servowristintake");
        servoClaw = hwmap.get(Servo.class, "servoclawintake");

        motorSliderHorizontal.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorSliderHorizontal.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motorSliderHorizontal.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorSliderHorizontal.setDirection(DcMotor.Direction.REVERSE);
    }
    Intake(HardwareMap hwmap) {
        dashboard = FtcDashboard.getInstance();

        pidIntake = new PID3(kP, kI, kD, 0);
        pidIntake.setOutputRange(-1.0, 1.0);
        pidIntake.setIntegralLimit(0.25);
        hardware(hwmap);
        servoClaw.setDirection(Servo.Direction.REVERSE);
    }
    void runSlider() {
        int currentPosition = motorSliderHorizontal.getCurrentPosition();
        double power = 0;
        switch (caseSliderHorizontal) {
            case RETRACTED:
                power = pidIntake.update(targetSliderRetracted, currentPosition);
                break;
            case EXTENDED:
                power = pidIntake.update(targetSliderExtended, currentPosition);
                break;
            case TRANSFER:
                power = pidIntake.update(targetSliderTransfer, currentPosition);
                break;
        }
        motorSliderHorizontal.setPower(power);
    }

    void runArm() {
        switch (caseArm) {
            case UP:
                servoArm.setPosition(positionArmUp);
                break;
            case DOWN:
                servoArm.setPosition(positionArmDown);
                break;
            case HOVERING:
                servoArm.setPosition(positionArmHovering);
                break;
            case TRANSFER:
                servoArm.setPosition(positionArmTransfer);
                break;
        }
    }

    void runWrist() {
        switch (caseWrist) {
            case HORIZONTAL:
                servoWrist.setPosition(positionWristHorizontal);
                break;
            case VERTICAL:
                servoWrist.setPosition(positionWristVertical);
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
