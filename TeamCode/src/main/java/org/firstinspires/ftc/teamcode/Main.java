package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Main extends LinearOpMode {
    Robot robot;

    void telemetry() {
        telemetry.addData("Outtake slider:", robot.outtake.motorSliderVertical.getCurrentPosition());
        telemetry.update();
    }

    @Override
    public void runOpMode() {
        robot = new Robot(hardwareMap);
        waitForStart();
        if(opModeIsActive()) {

        }
        while(opModeIsActive()) {
            robot.drive.move(gamepad1.left_stick_x, gamepad1.left_stick_y,
                    gamepad1.left_trigger, gamepad1.right_trigger);

            if(gamepad1.aWasPressed()) {
                robot.toggle();
            }

            if(Robot.states == Robot.stateMachine.PICKUP && gamepad1.bWasPressed())
                Robot.states = Robot.stateMachine.HOVERING;

            if(Robot.states == Robot.stateMachine.DUNKING && gamepad1.yWasPressed())
                switch (Outtake.caseSliderVerticalPos) {
                    case FULL:
                        Outtake.caseSliderVerticalPos = Outtake.sliderPos.MIDDLE;
                        break;
                    case MIDDLE:
                        Outtake.caseSliderVerticalPos = Outtake.sliderPos.FULL;
                        break;
                }

            if(Robot.states == Robot.stateMachine.HOVERING) {
                if(gamepad1.dpadUpWasPressed())
                    Intake.caseWrist = Intake.wrist.VERTICAL;
                if(gamepad1.dpadDownWasPressed())
                    Intake.caseWrist = Intake.wrist.HORIZONTAL;
            }

            robot.changeState();

            robot.run();

            telemetry();
        }
    }
}
