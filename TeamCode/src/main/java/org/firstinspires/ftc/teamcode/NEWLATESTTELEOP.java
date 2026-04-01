package org.firstinspires.ftc.teamcode;

import static com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior.BRAKE;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.BlockerServo;
import org.firstinspires.ftc.teamcode.mechanisms.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Transfer;

//@Disabled
@TeleOp
public class NEWLATESTTELEOP extends OpMode {
    MecanumDrive drive = new MecanumDrive();

    final double LAUNCHER_TARGET_POWER = 70;
    final double LAUNCHER_MIN_VELOCITY = 60;

    private DcMotorEx launcher; // adding = null at the end is just the same. it just tells the reader that it is empty and it wil be changed later.
    private  DcMotorEx intake;
    BlockerServo blocker = new BlockerServo();

    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING
    }

    private LaunchState launchState;

    @Override
    public void init() { //code to run ONCE the driver hits INIT
        drive.init(hardwareMap);

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        intake.setZeroPowerBehavior(BRAKE);
        intake.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        launcher.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        launcher.setZeroPowerBehavior(BRAKE);
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(300, 0, 0, 10)); //these are old values, we need new ones

        blocker.init(hardwareMap);

        launchState = launchState.IDLE;

        telemetry.addData("Status", "Initialized");
    }

    @Override
    public void init_loop() { //code to run REPEATEDLY after the driver hits INIT, but before they hit START

    }

    @Override
    public void start() { //code to run ONCE when the driver hits START
        //TODO make a preference modes for each driver, that is if they have a preference.
    }

    @Override
    public void loop() { //code to run REPEATEDLY after the driver hits START but before they hit STOP
        launch(gamepad1.y);

        if (gamepad1.b) {
            stopLaunch();
        }

        if (gamepad1.left_bumper) {
            intake.setPower(1.0);
        } else {
            intake.setPower(0.0);
        }

        if (gamepad1.a) { // TODO make it so that it will automatically lock the balls and with only clicking the fire button, it will open letting the balls out.
            blocker.setServoPos(0.5);
        } else {
            blocker.setServoPos(1.0);
        }

        drive.drive(-gamepad1.left_stick_y, gamepad1.left_stick_x, -gamepad1.right_stick_x); // robot oriented

        telemetry.addData("State", launchState);
        telemetry.addData("bumper", gamepad1.x);
        telemetry.addData("motorSpeed", getRPM(launcher.getVelocity()));
        telemetry.addData("target", LAUNCHER_TARGET_POWER);
        telemetry.addData("Heading", drive.getHeading());

    }

    public double getRPM(double ticks){
        return (ticks/28) * 60;
    }

    void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                launcher.setVelocity(LAUNCHER_TARGET_POWER);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                //feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                //if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    launchState = LaunchState.IDLE;
                //}
                break;
        }
    }

    void stopLaunch() {
        launcher.setVelocity(0);
        launchState = LaunchState.IDLE;
    }

}

//cleaned up code. i will clean it up even more later