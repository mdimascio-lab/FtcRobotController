package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
@Disabled
public class Gamepad extends OpMode {

    @Override
    public void init() {

    }

    @Override
    public void loop() {

        double speedForward = -gamepad1.left_stick_y / 2.0;
        double differenceX = gamepad1.left_stick_x - gamepad1.right_stick_x;
        double sumTriggers = gamepad1.left_trigger + gamepad1.right_trigger;

        telemetry.addData("xL", gamepad1.left_stick_x);
        telemetry.addData("yL", speedForward);

        telemetry.addData("xR", gamepad1.right_stick_x);
        telemetry.addData("yR", gamepad1.right_stick_y);
        telemetry.addData("a button", gamepad1.a);
        telemetry.addData("b button", gamepad1.b);

        telemetry.addData("difference x", differenceX);
        telemetry.addData("sum triggers", sumTriggers);
    }


}
