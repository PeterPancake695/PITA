package org.firstinspires.ftc.teamcode.servotester;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ServoRobot {
    FtcDashboard dashboard;
    public static double servoPosition = 0.0;
    Servo servo;

    public ServoRobot(HardwareMap hwmap) {
        dashboard = FtcDashboard.getInstance();
        servo = hwmap.get(Servo.class, "servo");
    }

    public void setServoPosition(){
        servo.setPosition(servoPosition);
    }
}
