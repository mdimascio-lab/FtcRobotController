package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.mechanisms.BlockerServo;

@TeleOp
public class LauncherTest extends OpMode {
    private DcMotorEx launcher;
    private DcMotorEx intake;
    BlockerServo blocker = new BlockerServo();

    final double LAUNCHER_TARGET_POWER = 70;
    final double LAUNCHER_MIN_VELOCITY = 60;
    final double STOP_SPEED = 0.0;

    @Override
    public void init() {
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(BRAKE);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setZeroPowerBehavior(BRAKE);
        launcher.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10));
        blocker.init(hardwareMap);

    }

    @Override
    public void loop() {
        if (gamepad1.y) {
            launcher.setVelocity(LAUNCHER_TARGET_POWER);
        } else if (gamepad1.b) {
            launcher.setVelocity(STOP_SPEED);
        }
        if (gamepad1.left_bumper) {
            intake.setPower(1.0);
        } else {
            intake.setPower(0.0);
        }

        if (gamepad1.a) {
            blocker.setServoPos(0.5);
        } else {
            blocker.setServoPos(1.0);
        }

        telemetry.addData("motorSpeed", launcher.getVelocity());
        telemetry.addData("target", LAUNCHER_TARGET_POWER);
        //telemetry.addData("blocker Position", .)
    }
}
