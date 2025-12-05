package org.firstinspires.ftc.teamcode;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.pedropathing.util.Timer;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

@Autonomous
public class    withIntakeAutoBLUUE extends OpMode{ // BRO THIS LOOKS SO GOOD OMG YOU COOKED
    private Follower follower;
    private Timer pathTimer, opModeTimer;

    public enum PathState {         // idk what im doing here but i guess it will work
        DRIVE_STARTtoSHOOT1,
        DRIVE_SHOOT1toBALLPILE1beforeC,             // C just meaning collected
        COLLECTION_BALLPILE1beforeCtoBALLPILE1afterC,
        DRIVE_BALLPILE1afterCtoSHOOT2,
        DRIVE_SHOOT2toBALLPILE2beforeC,
        COLLECTION_BALLPILE2beforeCtoBALLPILE2afterC,
        DRIVE_BALLPILE2afterCtoSHOOT3,
        DRIVE_SHOOT3toBALLPILE3beforeC,
        COLLECTION_BALLPILE3beforeCtoBALLPILE3afterC,
        DRIVE_BALLPILE3afterCtoSHOOT4,
        DRIVE_SHOOT4to90TELEOP,
        END_90TELEOP
    }

    PathState pathState;

    private final Pose startPose = new Pose(59.40720961281709,11.150867823765026, Math.toRadians(90));
    private final Pose shoot1 = new Pose(71.90387182910547, 71.71161548731642, Math.toRadians(132));
    private final Pose ballPile1BeforeC = new Pose(44.41121495327103, 35.18291054739653, Math.toRadians(0));
    private final Pose ballPile1AfterC = new Pose(23.647530040053404, 35.18291054739653, Math.toRadians(0));
    private final Pose shoot2 = new Pose(71.90387182910547, 71.90387182910547, Math.toRadians(132));
    private final Pose ballPile2BeforeC = new Pose(46.141522029372496, 59.79172229639519, Math.toRadians(0));
    private final Pose ballPile2AfterC = new Pose(24.03204272363151, 59.59946595460614, Math.toRadians(0));
    private final Pose shoot3 = new Pose(71.90387182910547, 72.09612817089453, Math.toRadians(132));
    private final Pose ballPile3BeforeC = new Pose(44.60347129506008, 84.20827770360481, Math.toRadians(0));    // we might not have enough time to shoot another time
    private final Pose ballPile3AfterC = new Pose(24.608811748998665, 84.01602136181576, Math.toRadians(0));
    private final Pose shoot4 = new Pose(71.71161548731642, 72.09612817089453, Math.toRadians(132));
    private final Pose ninetyToTeleOp = new Pose(47.10280373831775, 24.224299065420563, Math.toRadians(90));

    private PathChain driveStartPosShoot1Pos, driveShoot1PosBallPile1BeforeCPos, driveBallPile1BeforeCPosBallPile1AfterCPos, driveBallPile1AfterCPosShoot2Pos, driveShoot2PosBallPile2BeforeCPos, driveBallPile2BeforeCPosBallPile2AfterCPos, driveBallPile2AfterCPosShoot3Pos, driveShoot3PosBallPile3BeforeCPos, driveBallPile3BeforeCPosBallPile3AfterCpos, driveBallPile3AfterCPosShoot4Pos, driveShoot4PosNinetyToTeleOpPos; // uwa so long
    public void buildPaths() {  // i actually understand this
        driveStartPosShoot1Pos = follower.pathBuilder()
                .addPath(new BezierLine(startPose, shoot1))
                .setLinearHeadingInterpolation(startPose.getHeading(), shoot1.getHeading())
                .build();
        driveShoot1PosBallPile1BeforeCPos = follower.pathBuilder()
                .addPath(new BezierLine(shoot1, ballPile1BeforeC))
                .setLinearHeadingInterpolation(shoot1.getHeading(), ballPile1BeforeC.getHeading())
                .build();
        driveBallPile1BeforeCPosBallPile1AfterCPos = follower.pathBuilder()
                .addPath(new BezierLine(ballPile1BeforeC, ballPile1AfterC))
                .setLinearHeadingInterpolation(ballPile1BeforeC.getHeading(),ballPile1AfterC.getHeading())
                .build();
        driveBallPile1AfterCPosShoot2Pos = follower.pathBuilder()
                .addPath(new BezierLine(ballPile1AfterC, shoot2))
                .setLinearHeadingInterpolation(ballPile1AfterC.getHeading(), shoot2.getHeading())
                .build();
        driveShoot2PosBallPile2BeforeCPos = follower.pathBuilder()
                .addPath(new BezierLine(shoot2, ballPile2BeforeC))
                .setLinearHeadingInterpolation(shoot2.getHeading(), ballPile2BeforeC.getHeading())
                .build();
        driveBallPile2BeforeCPosBallPile2AfterCPos = follower.pathBuilder()
                .addPath(new BezierLine(ballPile2BeforeC, ballPile2AfterC))
                .setLinearHeadingInterpolation(ballPile2BeforeC.getHeading(), ballPile2AfterC.getHeading())
                .build();
        driveBallPile2AfterCPosShoot3Pos = follower.pathBuilder()
                .addPath(new BezierLine(ballPile2AfterC, shoot3))
                .setLinearHeadingInterpolation(ballPile2AfterC.getHeading(),shoot3.getHeading())
                .build();
        driveShoot3PosBallPile3BeforeCPos = follower.pathBuilder()
                .addPath(new BezierLine(shoot3, ballPile3BeforeC))
                .setLinearHeadingInterpolation(shoot3.getHeading(), ballPile3BeforeC.getHeading())  // genuine flow state
                .build();
        driveBallPile3BeforeCPosBallPile3AfterCpos = follower.pathBuilder()
                .addPath(new BezierLine(ballPile3BeforeC, ballPile3AfterC))
                .setLinearHeadingInterpolation(ballPile3BeforeC.getHeading(), ballPile3AfterC.getHeading())
                .build();
        driveBallPile3AfterCPosShoot4Pos = follower.pathBuilder()
                .addPath(new BezierLine(ballPile3AfterC, shoot4))
                .setLinearHeadingInterpolation(ballPile3AfterC.getHeading(), shoot4.getHeading())
                .build();
        driveShoot4PosNinetyToTeleOpPos = follower.pathBuilder()
                .addPath(new BezierLine(shoot4, ninetyToTeleOp))
                .setLinearHeadingInterpolation(shoot4.getHeading(), ninetyToTeleOp.getHeading())
                .build();   //took way too long :sob:
    }

    public void statePathUpdate() { // i somewhat understand this?
        switch (pathState) {
            case DRIVE_STARTtoSHOOT1:
                follower.followPath(driveStartPosShoot1Pos, true);
                setPathState(PathState.DRIVE_SHOOT1toBALLPILE1beforeC);
                break;
            case DRIVE_SHOOT1toBALLPILE1beforeC:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    follower.followPath(driveShoot1PosBallPile1BeforeCPos, true);
                    setPathState(PathState.COLLECTION_BALLPILE1beforeCtoBALLPILE1afterC);
                }
                break;
            case COLLECTION_BALLPILE1beforeCtoBALLPILE1afterC:
                if (!follower.isBusy()) {
                    follower.followPath(driveBallPile1BeforeCPosBallPile1AfterCPos);
                    setPathState(PathState.DRIVE_BALLPILE1afterCtoSHOOT2);
                }
                break;
            case DRIVE_BALLPILE1afterCtoSHOOT2:
                if (!follower.isBusy()) {
                    follower.followPath(driveBallPile1AfterCPosShoot2Pos);
                    setPathState(PathState.DRIVE_SHOOT2toBALLPILE2beforeC);
                }
                break;
            case DRIVE_SHOOT2toBALLPILE2beforeC:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    follower.followPath(driveShoot2PosBallPile2BeforeCPos);
                    setPathState(PathState.COLLECTION_BALLPILE2beforeCtoBALLPILE2afterC);
                }
                break;
            case COLLECTION_BALLPILE2beforeCtoBALLPILE2afterC:
                if (!follower.isBusy()) {
                    follower.followPath(driveBallPile2BeforeCPosBallPile2AfterCPos);
                    setPathState(PathState.DRIVE_BALLPILE2afterCtoSHOOT3);
                }
                break;
            case DRIVE_BALLPILE2afterCtoSHOOT3:
                if (!follower.isBusy()) {
                    follower.followPath(driveBallPile2AfterCPosShoot3Pos);
                    setPathState(PathState.DRIVE_SHOOT3toBALLPILE3beforeC);
                }
                break;
            case DRIVE_SHOOT3toBALLPILE3beforeC:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    follower.followPath(driveShoot3PosBallPile3BeforeCPos);
                    setPathState(PathState.COLLECTION_BALLPILE3beforeCtoBALLPILE3afterC);
                }
                break;
            case COLLECTION_BALLPILE3beforeCtoBALLPILE3afterC:
                if (!follower.isBusy()) {
                    follower.followPath(driveBallPile3BeforeCPosBallPile3AfterCpos);
                    setPathState(PathState.DRIVE_BALLPILE3afterCtoSHOOT4);
                }
                break;
            case DRIVE_BALLPILE3afterCtoSHOOT4:
                if (!follower.isBusy()) {
                    follower.followPath(driveBallPile3AfterCPosShoot4Pos);
                    setPathState(PathState.DRIVE_SHOOT4to90TELEOP);
                }
                break;
            case DRIVE_SHOOT4to90TELEOP:
                if (!follower.isBusy() && pathTimer.getElapsedTimeSeconds() > 5) {
                    follower.followPath(driveShoot4PosNinetyToTeleOpPos);
                    setPathState(PathState.END_90TELEOP);
                }
                break;
            case END_90TELEOP:
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

    @Override
    public void init() {
        pathState = PathState.DRIVE_STARTtoSHOOT1;
        pathTimer = new Timer();
        opModeTimer = new Timer();
        follower = Constants.createFollower(hardwareMap);

        buildPaths();
        follower.setPose(startPose);
    }

    public void start() {
        opModeTimer.resetTimer();
        setPathState(pathState);
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
    }
}



