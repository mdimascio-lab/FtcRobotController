package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled 
@TeleOp(name="hello world", group="test")
public class HellowWorld extends OpMode {

    @Override
    public void init() {
        telemetry.addData("Hello", "world");
    }

    @Override
    public void loop() {

    }
}

