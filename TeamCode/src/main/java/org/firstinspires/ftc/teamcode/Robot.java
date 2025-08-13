package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.concurrent.TimeUnit;

public class Robot {
    public enum stateMachine {
        RETRACTED,
        HOVERING,
        PICKUP,
        DUNKING
    }

    public static stateMachine states;

    ElapsedTime pickupTimer;
    ElapsedTime transferTimer;

    Drive drive;
    Intake intake;
    Outtake outtake;
    Robot(HardwareMap hwmap) {
        states = stateMachine.RETRACTED;

        pickupTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        pickupTimer.startTime();
        transferTimer = new ElapsedTime(ElapsedTime.Resolution.MILLISECONDS);
        transferTimer.startTime();

        drive = new Drive(hwmap);
        intake = new Intake(hwmap);
        outtake = new Outtake(hwmap);
    }
    public void toggle() {
        switch (states){
            case RETRACTED:
                states = stateMachine.HOVERING;
                break;
            case HOVERING:
                states = stateMachine.PICKUP;
                pickupTimer.reset();
                break;
            case PICKUP:
                states = stateMachine.DUNKING;
                transferTimer.reset();
                break;
            case DUNKING:
                states = stateMachine.RETRACTED;
                break;
        }
    }

    public void changeState() {
        switch (states) {
            case RETRACTED:
                Intake.caseSliderHorizontal = Intake.sliderHorizontal.RETRACTED;
                Intake.caseWrist = Intake.wrist.HORIZONTAL;
                Intake.caseArm = Intake.arm.UP;
                Intake.caseClaw = Intake.claw.CLOSED;

                Outtake.caseWrist = Outtake.wrist.LEFT;
                Outtake.caseArm = Outtake.arm.LEFT;
                Outtake.caseSliderVertical = Outtake.sliderVertical.RETRACTED;
                Outtake.caseClaw = Outtake.claw.OPEN;
                break;

            case HOVERING:
                Intake.caseSliderHorizontal = Intake.sliderHorizontal.EXTENDED;
                Intake.caseArm = Intake.arm.HOVERING;
                Intake.caseClaw = Intake.claw.OPEN;
                break;

            case PICKUP: // timer might not work lmfao so might have to change timing idfk
                if(pickupTimer.time(TimeUnit.MILLISECONDS) < 200)
                    Intake.caseArm = Intake.arm.DOWN;

                if(pickupTimer.time(TimeUnit.MILLISECONDS) > 250
                        && pickupTimer.time(TimeUnit.MILLISECONDS) < 300)
                    Intake.caseClaw = Intake.claw.CLOSED;

                if(pickupTimer.time(TimeUnit.MILLISECONDS) > 350
                        && pickupTimer.time(TimeUnit.MILLISECONDS) < 400)
                    Intake.caseArm = Intake.arm.HOVERING;
                break;

            case DUNKING: // timing problem applies here too ESPECIALLY HERE
                if(transferTimer.time(TimeUnit.MILLISECONDS) < 2000) {
                    Intake.caseWrist = Intake.wrist.HORIZONTAL;
                    Intake.caseArm = Intake.arm.TRANSFER;
                    Intake.caseSliderHorizontal = Intake.sliderHorizontal.TRANSFER;
                }

                if(transferTimer.time(TimeUnit.MILLISECONDS) > 2150
                        && transferTimer.time(TimeUnit.MILLISECONDS) < 2200)
                    Outtake.caseClaw = Outtake.claw.CLOSED;

                if(transferTimer.time(TimeUnit.MILLISECONDS) > 2250
                        && transferTimer.time(TimeUnit.MILLISECONDS) < 2300)
                    Intake.caseClaw = Intake.claw.OPEN;

                if(transferTimer.time(TimeUnit.MILLISECONDS) > 2350
                        && transferTimer.time(TimeUnit.MILLISECONDS) < 4300) {
                    Outtake.caseSliderVertical = Outtake.sliderVertical.EXTENDED;
                    Outtake.caseArm = Outtake.arm.RIGHT;
                    Outtake.caseWrist = Outtake.wrist.RIGHT;
                }
                break;
        }
    }

    public void run() {
        intake.runSlider();
        intake.runArm();
        intake.runWrist();
        intake.runClaw();

        outtake.runSlider();
        outtake.runArm();
        outtake.runWrist();
        outtake.runClaw();
    }
}
