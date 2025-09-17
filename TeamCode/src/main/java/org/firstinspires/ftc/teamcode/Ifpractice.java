package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class Ifpractice extends OpMode {

    @Override
    public void init() {

    }

    @Override
    public void loop() {
        boolean  aButton = gamepad1.a;

        if (aButton) {
            telemetry.addData("A Button", "pressed!");
        }
        else {
            telemetry.addData("A Button", "NOT pressed!");
        }
        telemetry.addData("A Button State", aButton);


    }
}

/*
double leftY = gamepad1.left_stick_y

        if (leftY < 0) {
            telemetry.addData("Left Stick", "is Negative");
        }
         else if (leftY > 0.5) {
            telemetry.addData("Left Stick", "Greater than 50%");
         }

        else if (leftY > 0) {
            telemetry.addData("Left Stick", "Is greater than 8");
        }

        else {
            telemetry.addData("Left Stick", "is Zero!");
        }
        telemetry.addData("Left Stick value", leftY);
 */