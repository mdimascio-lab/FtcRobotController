package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@TeleOp
public class launcherspeedtest extends OpMode {

    public DcMotorEx flywheelMotor;

    private float GEAR_RATIO = 52/68;
    public double LAUNCHER_TARGET_POWER = 0.7;
    public double LAUNCHER_MIN_VELOCITY = 0.6;
    double[] speedMotor = {0.1, 0.01};
    int stepIndex = 1;

    @Override
    public void loop() {
        if (gamepad1.aWasPressed()) {
            stepIndex = (stepIndex + 1) % speedMotor.length;
        }

        if (gamepad1.dpadUpWasPressed()) {
            LAUNCHER_TARGET_POWER += speedMotor[stepIndex];
        }

        if (gamepad1.dpadDownWasPressed()) {
            LAUNCHER_TARGET_POWER -= speedMotor[stepIndex];
        }
        flywheelMotor.setPower(LAUNCHER_TARGET_POWER);

        telemetry.addData("Current RPM", "%.2f", getRPM(flywheelMotor.getVelocity()));
        telemetry.addData("Current RPM", "%.2f", ((flywheelMotor.getVelocity()/28.0*60)*(52.0/68.0)));
        telemetry.addData("Set Speed", "%.4f (D-Pad U/D", LAUNCHER_TARGET_POWER);
        telemetry.addData("Speed Size", "%.4f (A Button", speedMotor[stepIndex]);
    }

    @Override
    public void init() {
        flywheelMotor = hardwareMap.get(DcMotorEx.class, "launcher");
        flywheelMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        flywheelMotor.setDirection(DcMotorSimple.Direction.REVERSE);
    }
    public double getRPM(double ticks){
        return (((ticks/28.0) * 60)*(52.0/68.0));
    }
}
