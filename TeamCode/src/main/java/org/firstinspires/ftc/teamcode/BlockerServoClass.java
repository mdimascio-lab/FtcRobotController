package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.BlockerServo;

@TeleOp
public class BlockerServoClass extends OpMode{
    BlockerServo blocker = new BlockerServo();
    @Override
    public void init() {
        blocker.init(hardwareMap);
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            blocker.setServoPos(0.5); // full left. mid point 0.5
        }
        else {
            blocker.setServoPos(1.0); // full right
        }

    }
}
