package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Transfer {
    private CRServo leftFeederServo, rightFeederServo;

    final double FEED_TIME_SECONDS = 2.0;

    ElapsedTime feedTimer = new ElapsedTime();

    public void init(HardwareMap hardwareMap) {
        leftFeederServo =  hardwareMap.get(CRServo.class, "left_feeder");
        rightFeederServo =  hardwareMap.get(CRServo.class, "right_feeder");
    }

    public void transferRight() {
        rightFeederServo.setPower(1.0);
        feedTimer.reset();
        if (feedTimer.seconds() > FEED_TIME_SECONDS) {
            rightFeederServo.setPower(0);
        }
    }
    public void transferLeft() {
        leftFeederServo.setPower(1.0);
        feedTimer.reset();
        if (feedTimer.seconds() > FEED_TIME_SECONDS) {
            leftFeederServo.setPower(0);
        }
    }
    public void transferAll() {
        leftFeederServo.setPower(1.0);
        rightFeederServo.setPower(1.0);
        feedTimer.reset();
        if (feedTimer.seconds() > FEED_TIME_SECONDS) {
            leftFeederServo.setPower(0);
            rightFeederServo.setPower(0);
        }
    }
}
