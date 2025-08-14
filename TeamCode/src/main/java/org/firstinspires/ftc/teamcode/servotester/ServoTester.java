package org.firstinspires.ftc.teamcode.servotester;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class ServoTester extends LinearOpMode {
    ServoRobot robot;

    @Override
    public void runOpMode() {
        robot = new ServoRobot(hardwareMap);

        waitForStart();
        if(opModeIsActive()) {

        }
        while(opModeIsActive()) {
            robot.setServoPosition();
        }
    }
}
