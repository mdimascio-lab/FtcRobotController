package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.util.ElapsedTime;


import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class TriangleAutoRedShooting extends OpMode {
    // -----------  shooter loigc ---------------

    private DcMotorEx launcher = null;
    private CRServo leftFeeder = null;
    private CRServo rightFeeder = null;


    final double LAUNCHER_TARGET_VELOCITY = 1430; // PREV. 1530
    final double LAUNCHER_VELOCITY_MARGIN = 50;
    final double LAUNCHER_MIN_VELOCITY = LAUNCHER_TARGET_VELOCITY-LAUNCHER_VELOCITY_MARGIN; // prev.1170
    final double LAUNCHER_MAX_VELOCITY = LAUNCHER_TARGET_VELOCITY+LAUNCHER_VELOCITY_MARGIN;

    private enum LaunchState {
        IDLE,
        SPIN_UP,
        LAUNCH,
        LAUNCHING,
        WAIT_BETWEEN_SHOTS
    }

    private LaunchState launchState;
    final double FEED_TIME_SECONDS = 0.20; //The feeder servos run this long when a shot is requested.
    final double STOP_SPEED = 0.0; //We send this power to the servos when we want them to stop.
    final double FULL_SPEED = 1.0;
    final double BETWEEN_SHOTS_DELAY = 1.0; // prev was 2.0


    private Follower follower;
    private Timer pathTimer, opModeTimer;
    ElapsedTime feederTimer = new ElapsedTime();

    public enum PathState {
        // START POSITON_END POSITION `
        // DRIVE > MOVEMOVENT STATE
        // SHOOT > ATTEMPT TO SCORE THE ARTIFACT
        DRIVE_STARTPOS_SHOOT_POS,
        SHOOT_PRELOAD,

        DRIVE_SHOOTPOS_ENDPOS
    }

    // ------------- PATH LOGIC ------------------
    PathState pathState;

    private final Pose startPose = new Pose(81.7089452603, 9.612817089452607, Math.toRadians(90));
    private final Pose shootPose = new Pose(85.35652173913043, 94.53913043478263, Math.toRadians(40)); // TODO FIX PLEASE IT'S WRONG previously 70.75033377837116, 73.24966622162884

    private final Pose endPose = new Pose(86.89986648865154, 57.86915887850468, Math.toRadians(90)); //TODO AND THIS TOO


    private PathChain driveStartPosShootPos, driveShootPosEndPos;

    public void buildPaths() {
        // put in coordinates for starting pose > ending pose
        driveStartPosShootPos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shootPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), shootPose.getHeading())
                .build();
        driveShootPosEndPos = follower.pathBuilder()
                .addPath(new BezierLine(shootPose, endPose))
                .setLinearHeadingInterpolation(shootPose.getHeading(), endPose.getHeading())
                .build();


    }

    public void statePathUpdate() {
        switch (pathState) {
            case DRIVE_STARTPOS_SHOOT_POS:
                follower.followPath(driveStartPosShootPos, true);
                setPathState(PathState.SHOOT_PRELOAD);
                launch(false);
                // start spinning up timer
                // reset the timer & make new state
                break;
            case SHOOT_PRELOAD:
                // check is follower done it's path?
                // and check that 5 seconds has elapsed
                launch(false);

                if (!follower.isBusy()) {
                    // At shoot position now → FIRE
                    launch(true);
                }
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 10) {
                    follower.followPath(driveShootPosEndPos, true);
                    setPathState(PathState.DRIVE_SHOOTPOS_ENDPOS);
                    launchState = LaunchState.IDLE;
                    launch(false);
                }
                break;
            case DRIVE_SHOOTPOS_ENDPOS:
                // all done!
                if (!follower.isBusy()) {
                    telemetry.addLine("Done all Paths");
                }
            default:
                telemetry.addLine("No State Commanded");
                break;
        }

    }

    public void setPathState(PathState newState) {
        pathState = newState;
        pathTimer.resetTimer();
    }

    void launch(boolean shotRequested) {
        switch (launchState) {
            case IDLE:
                if (shotRequested) {
                    launchState = LaunchState.SPIN_UP;
                }
                break;
            case SPIN_UP:
                launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
                if (launcher.getVelocity() > LAUNCHER_MIN_VELOCITY && launcher.getVelocity() < LAUNCHER_MAX_VELOCITY) {
                    launchState = LaunchState.LAUNCH;
                }
                break;
            case LAUNCH:
                leftFeeder.setPower(FULL_SPEED);
                rightFeeder.setPower(FULL_SPEED);
                feederTimer.reset();
                launchState = LaunchState.LAUNCHING;
                break;
            case LAUNCHING:
                if (feederTimer.seconds() > FEED_TIME_SECONDS) {
                    leftFeeder.setPower(STOP_SPEED);
                    rightFeeder.setPower(STOP_SPEED);

                    // Start the pause timer
                    feederTimer.reset();
                    launchState = LaunchState.WAIT_BETWEEN_SHOTS;
                }
                break;
            case WAIT_BETWEEN_SHOTS:
                // Simply wait here without feeding or shooting
                if (feederTimer.seconds() > BETWEEN_SHOTS_DELAY) {
                    // After the wait time, resume spin-up for next shot
                    launchState = LaunchState.SPIN_UP;
                }
                break;
        }
    }


    @Override
    public void init() {

        launcher = hardwareMap.get(DcMotorEx.class, "launcher");
        leftFeeder = hardwareMap.get(CRServo .class, "left_feeder");
        rightFeeder = hardwareMap.get(CRServo.class, "right_feeder");
        launcher.setPIDFCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, new PIDFCoefficients(265, 0, 0, 12.5));


        leftFeeder.setDirection(DcMotorSimple.Direction.REVERSE);

        pathState = PathState.DRIVE_STARTPOS_SHOOT_POS;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);
        // TODO add in any other init mechanicms
        launcher = hardwareMap.get(DcMotorEx.class, "launcher");

        buildPaths();
        follower.setPose(startPose);

        launchState = LaunchState.IDLE;
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
        launcher.setVelocity(LAUNCHER_TARGET_VELOCITY);
    }

    @Override
    public void loop(){
        follower.update();
        statePathUpdate();

        telemetry.addData("path state", pathState.toString());
        telemetry.addData("x", follower.getPose().getX());
        telemetry.addData("y", follower.getPose().getY());
        telemetry.addData("heading", Math.toRadians(follower.getHeading()));
        telemetry.addData("Path time", pathTimer.getElapsedTimeSeconds());
        telemetry.addData("launcher", launcher.getVelocity());
    }
}


