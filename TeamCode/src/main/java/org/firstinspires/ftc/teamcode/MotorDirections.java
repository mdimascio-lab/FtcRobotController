

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
@Disabled
@TeleOp
public class MotorDirections extends LinearOpMode {
    DcMotor frontLeftDrive;
    DcMotor frontRightDrive;
    DcMotor backLeftDrive;
    DcMotor backRightDrive;
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();
        frontLeftDrive = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRightDrive = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeftDrive = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRightDrive = hardwareMap.get(DcMotor.class, "back_right_drive");

        // We set the left motors in reverse which is needed for drive trains where the left
        // motors are opposite to the right ones.
        backLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        frontLeftDrive.setDirection(DcMotor.Direction.REVERSE);
        while(opModeIsActive()){
            if (gamepad1.a){
                frontLeftDrive.setPower(1.0);
                backLeftDrive.setPower(0.0);
                frontRightDrive.setPower(0.0);
                backRightDrive.setPower(0.0);
                telemetry.addLine("frontLeft");
            } else if (gamepad1.x) {
                backLeftDrive.setPower(1.0);
                frontLeftDrive.setPower(0.0);
                frontRightDrive.setPower(0.0);
                backRightDrive.setPower(0.0);
                telemetry.addLine("backLeft");
            } else if (gamepad1.y) {
                frontRightDrive.setPower(1.0);
                frontLeftDrive.setPower(0.0);
                backLeftDrive.setPower(0.0);
                backRightDrive.setPower(0.0);
                telemetry.addLine("frontRight");
            } else if (gamepad1.b) {
                backRightDrive.setPower(1.0);
                frontLeftDrive.setPower(0.0);
                backLeftDrive.setPower(0.0);
                frontRightDrive.setPower(0.0);
                telemetry.addLine("backRight");
            } else {
                frontLeftDrive.setPower(0.0);
                backLeftDrive.setPower(0.0);
                frontRightDrive.setPower(0.0);
                backRightDrive.setPower(0.0);
            }
            telemetry.update();
        }
    }
}
