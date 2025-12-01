package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.List;

@Autonomous(name = "Auton", group = "Auto")
public class AutonBlue extends OpMode {

    private Hardware robot;
    private Follower follower;


    public PathChain scorePreloadBlue, scorePreloadRed;
    public PathChain toFirstChainBlue, toFirstChainRed;
    public PathChain grabFirstBlue, grabFirstRed;
    public PathChain grabSecondBlue, grabSecondRed;
    public PathChain grabThirdBlue, grabThirdRed;
    public PathChain scoreFirstChainBlue, scoreFirstChainRed;
    public PathChain toSecondChainBlue, toSecondChainRed;
    public PathChain grabFourthBlue, grabFourthRed;
    public PathChain grabFifthBlue, grabFifthRed;
    public PathChain grabSixthBlue, grabSixthRed;
    public PathChain scoreSecondChainBlue, scoreSecondChainRed;
    public PathChain leaveBlue, leaveRed;

    private int pathState, actionState;
    private Timer pathTimer, actionTimer;

    public static Pose startingPose = new Pose(15,114,Math.toRadians(0));
    // ---------- RED POSES ----------
    public static Pose startingPoseRed = new Pose(129,114,Math.toRadians(180));
    public static Pose scorePreloadPoseRed = new Pose(104.6,107,Math.toRadians(116.6));
    public static Pose controlScorePoseRed = new Pose(115.3,117.4);
    public static Pose controlToFirstRed = new Pose(88.7,98);
    public static Pose toFirstRedChainPose = new Pose(100,86,Math.toRadians(0));
    public static Pose grabFirstPoseRed = new Pose(107,86,Math.toRadians(0));
    public static Pose grabSecondPoseRed = new Pose(113,86,Math.toRadians(0));
    public static Pose grabThirdPoseRed = new Pose(127,86,Math.toRadians(0));
    public static Pose scoreFirstPoseRed = new Pose(93,100,Math.toRadians(260));
    public static Pose toSecondStartRed = new Pose(96,96);
    public static Pose toSecondMidRed = new Pose(76.6,80);
    public static Pose toSecondEndRed = new Pose(98,60);
    public static Pose grabFourthPoseRed = new Pose(107,60);
    public static Pose grabFifthPoseRed = new Pose(113,60);
    public static Pose grabSixthPoseRed = new Pose(120,60);
    public static Pose scoreSecondPoseRed = new Pose(93,100);
    public static Pose leavePoseRed = new Pose(124,72);

    // ---------- BLUE POSES ----------
    public static Pose startingPoseBlue = mirror(startingPoseRed);
    public static Pose scorePreloadPoseBlue = mirror(scorePreloadPoseRed);
    public static Pose controlScorePoseBlue = mirror(controlScorePoseRed);
    public static Pose controlToFirstBlue = mirror(controlToFirstRed);
    public static Pose toFirstBlue = mirror(toFirstRedChainPose);
    public static Pose grabFirstPoseBlue = mirror(grabFirstPoseRed);
    public static Pose grabSecondPoseBlue = mirror(grabSecondPoseRed);
    public static Pose grabThirdPoseBlue = mirror(grabThirdPoseRed);
    public static Pose scoreFirstPoseBlue = mirror(scoreFirstPoseRed);
    public static Pose toSecondStartBlue = mirror(toSecondStartRed);
    public static Pose toSecondMidBlue = mirror(toSecondMidRed);
    public static Pose toSecondEndBlue = mirror(toSecondEndRed);
    public static Pose grabFourthPoseBlue = mirror(grabFourthPoseRed);
    public static Pose grabFifthPoseBlue = mirror(grabFifthPoseRed);
    public static Pose grabSixthPoseBlue = mirror(grabSixthPoseRed);
    public static Pose scoreSecondPoseBlue = mirror(scoreSecondPoseRed);
    public static Pose leavePoseBlue = mirror(leavePoseRed);

    private boolean isRed = false;
    boolean caseInitialized = false;

    private PathChain scorePreload;
    private PathChain toFirstChain;
    private PathChain grabFirstChain, grabSecondChain, grabThirdChain;
    private PathChain grabFourthChain, grabFifthChain, grabSixthChain;
    private PathChain scoreFirstChain;
    private PathChain toSecondChain;
    private PathChain scoreSecondChain;
    private PathChain leave;


    private Methods methods = new Methods();
    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose);
        follower.update();
        Values.reset();

        robot = new Hardware(hardwareMap);
        robot.init();
        robot.limelight.start();
        buildPaths();
        robot.turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        pathTimer = new Timer();
        actionTimer = new Timer();
    }


    private void buildPaths() {

        // =============== BLUE AUTON ===============

        scorePreloadBlue = follower.pathBuilder()
                .addPath(new BezierCurve(startingPoseBlue, controlScorePoseBlue, scorePreloadPoseBlue))
                .setLinearHeadingInterpolation(
                        startingPoseBlue.getHeading(),
                        scorePreloadPoseBlue.getHeading())
                .build();

        toFirstChainBlue = follower.pathBuilder()
                .addPath(new BezierCurve(scorePreloadPoseBlue, controlToFirstBlue, toFirstBlue))
                .setLinearHeadingInterpolation(
                        scorePreloadPoseBlue.getHeading(),
                        toFirstBlue.getHeading())
                .build();

        grabFirstBlue = follower.pathBuilder()
                .addPath(new BezierLine(toFirstBlue, grabFirstPoseBlue))
                .setTangentHeadingInterpolation()
                .build();

        grabSecondBlue = follower.pathBuilder()
                .addPath(new BezierLine(grabFirstPoseBlue, grabSecondPoseBlue))
                .setTangentHeadingInterpolation()
                .build();

        grabThirdBlue = follower.pathBuilder()
                .addPath(new BezierLine(grabSecondPoseBlue, grabThirdPoseBlue))
                .setTangentHeadingInterpolation()
                .build();

        scoreFirstChainBlue = follower.pathBuilder()
                .addPath(new BezierCurve(grabThirdPoseBlue, scoreFirstPoseBlue))
                .setLinearHeadingInterpolation(
                        grabThirdPoseBlue.getHeading(),
                        scoreFirstPoseBlue.getHeading())
                .build();

        toSecondChainBlue = follower.pathBuilder()
                .addPath(new BezierCurve(toSecondStartBlue, toSecondMidBlue, toSecondEndBlue))
                .setLinearHeadingInterpolation(
                        toSecondStartBlue.getHeading(),
                        toSecondEndBlue.getHeading())
                .build();

        grabFourthBlue = follower.pathBuilder()
                .addPath(new BezierLine(toSecondEndBlue, grabFourthPoseBlue))
                .setTangentHeadingInterpolation()
                .build();

        grabFifthBlue = follower.pathBuilder()
                .addPath(new BezierLine(grabFourthPoseBlue, grabFifthPoseBlue))
                .setTangentHeadingInterpolation()
                .build();

        grabSixthBlue = follower.pathBuilder()
                .addPath(new BezierLine(grabFifthPoseBlue, grabSixthPoseBlue))
                .setTangentHeadingInterpolation()
                .build();

        scoreSecondChainBlue = follower.pathBuilder()
                .addPath(new BezierLine(grabSixthPoseBlue, scoreSecondPoseBlue))
                .setLinearHeadingInterpolation(
                        grabSixthPoseBlue.getHeading(),
                        scoreSecondPoseBlue.getHeading())
                .build();

        leaveBlue = follower.pathBuilder()
                .addPath(new BezierLine(scoreSecondPoseBlue, leavePoseBlue))
                .setTangentHeadingInterpolation()
                .build();




        // =============== RED AUTON ===============

        scorePreloadRed = follower.pathBuilder()
                .addPath(new BezierCurve(startingPoseRed, controlScorePoseRed, scorePreloadPoseRed))
                .setLinearHeadingInterpolation(
                        startingPoseRed.getHeading(),
                        scorePreloadPoseRed.getHeading())
                .build();

        toFirstChainRed = follower.pathBuilder()
                .addPath(new BezierCurve(scorePreloadPoseRed, controlToFirstRed, toFirstRedChainPose))
                .setLinearHeadingInterpolation(
                        scorePreloadPoseRed.getHeading(),
                        toFirstRedChainPose.getHeading())
                .build();

        grabFirstRed = follower.pathBuilder()
                .addPath(new BezierLine(toFirstRedChainPose, grabFirstPoseRed))
                .setTangentHeadingInterpolation()
                .build();

        grabSecondRed = follower.pathBuilder()
                .addPath(new BezierLine(grabFirstPoseRed, grabSecondPoseRed))
                .setTangentHeadingInterpolation()
                .build();

        grabThirdRed = follower.pathBuilder()
                .addPath(new BezierLine(grabSecondPoseRed, grabThirdPoseRed))
                .setTangentHeadingInterpolation()
                .build();

        scoreFirstChainRed = follower.pathBuilder()
                .addPath(new BezierCurve(grabThirdPoseRed, scoreFirstPoseRed))
                .setLinearHeadingInterpolation(
                        grabThirdPoseRed.getHeading(),
                        scoreFirstPoseRed.getHeading())
                .build();

        toSecondChainRed = follower.pathBuilder()
                .addPath(new BezierCurve(toSecondStartRed, toSecondMidRed, toSecondEndRed))
                .setLinearHeadingInterpolation(
                        toSecondStartRed.getHeading(),
                        toSecondEndRed.getHeading())
                .build();

        grabFourthRed = follower.pathBuilder()
                .addPath(new BezierLine(toSecondEndRed, grabFourthPoseRed))
                .setTangentHeadingInterpolation()
                .build();

        grabFifthRed = follower.pathBuilder()
                .addPath(new BezierLine(grabFourthPoseRed, grabFifthPoseRed))
                .setTangentHeadingInterpolation()
                .build();

        grabSixthRed = follower.pathBuilder()
                .addPath(new BezierLine(grabFifthPoseRed, grabSixthPoseRed))
                .setTangentHeadingInterpolation()
                .build();

        scoreSecondChainRed = follower.pathBuilder()
                .addPath(new BezierLine(grabSixthPoseRed, scoreSecondPoseRed))
                .setLinearHeadingInterpolation(
                        grabSixthPoseRed.getHeading(),
                        scoreSecondPoseRed.getHeading())
                .build();

        leaveRed = follower.pathBuilder()
                .addPath(new BezierLine(scoreSecondPoseRed, leavePoseRed))
                .setTangentHeadingInterpolation()
                .build();
    }


    @Override
    public void init_loop() {

        if (gamepad1.aWasPressed()) {
            Values.team = "blue";
            startingPose = new Pose(15, 112, Math.toRadians(0));
        } else if (gamepad1.bWasPressed()) {
            Values.team = "red";
            startingPose = new Pose(129, 112, Math.toRadians(180));
        }

        isRed = Values.team.equals("red");
        telemetry.addData("Team: ", Values.team);
        telemetry.addData("Starting Pose", startingPose);
        telemetry.addData("connection",robot.limelight.isConnected());
        telemetry.addData("ll",robot.limelight.getLatestResult().getFiducialResults());
        telemetry.addData("status", robot.limelight.getStatus());

    }


    @Override
    public void start() {

        follower.setStartingPose(startingPose);

        scorePreload     = isRed ? scorePreloadRed     : scorePreloadBlue;
        toFirstChain     = isRed ? toFirstChainRed     : toFirstChainBlue;
        grabFirstChain = isRed ? grabFirstRed : grabFirstBlue;
        grabSecondChain = isRed ? grabSecondRed : grabSecondBlue;
        grabThirdChain = isRed ? grabThirdRed : grabThirdBlue;
        scoreFirstChain  = isRed ? scoreFirstChainRed  : scoreFirstChainBlue;
        toSecondChain    = isRed ? toSecondChainRed    : toSecondChainBlue;
        grabFourthChain = isRed ? grabFourthRed : grabFourthBlue;
        grabFifthChain = isRed ? grabFifthRed : grabFifthBlue;
        grabSixthChain = isRed ? grabSixthRed : grabSixthBlue;
        scoreSecondChain = isRed ? scoreSecondChainRed : scoreSecondChainBlue;
        leave            = isRed ? leaveRed            : leaveBlue;

        setPathState(0);
        actionState=0;
        actionTimer.resetTimer();
    }

    @Override
    public void loop() {
        follower.update();
        autonomousPathUpdate();
        Values.motif=getMotif(robot.limelight,telemetry);
        telemetry.addData("state",pathState);
        telemetry.addData("motif",Values.motif);
        telemetry.addData("target",Values.spindexerConstants.spindexerPosition);
        telemetry.addData("curr",robot.spindexer.getCurrentPosition());
        telemetry.addData("timer",pathTimer.getElapsedTimeSeconds());
        methods.positionPID(robot.turret,Values.turretConstants.turretPosition,"turret");
        methods.velocityPID(robot.flywheel,Values.flywheelConstants.flywheelVelocity,"flywheel");
        methods.positionPID(robot.spindexer,Values.spindexerConstants.spindexerPosition, "spindexer");
        telemetry.update();
    }

    public void transferStop(){
        robot.transfer.setPosition(Values.transferBeltStop);
        robot.transferEngage.setPosition(Values.transferDisengage);
    }
    public void transferBelt(){
        robot.transfer.setPosition(Values.transferBeltStart);
        robot.transferEngage.setPosition(Values.transferEngage);
    }
    public void transferKick(){
        robot.transfer.setPosition(Values.transferBeltStart);
        robot.transferEngage.setPosition(Values.transferKick);
    }


    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                if (Values.team.equals("red")) {
                    Values.turretConstants.turretPosition = -440;
                }else{
                    Values.turretConstants.turretPosition=440;
                }
                follower.setMaxPower(1);
                Values.motif=getMotif(robot.limelight,telemetry);
//                switch (Values.motif) {
//                    case PPG:
//                    case PGP:
//                        Values.spindexerConstants.sA = 400000;
//                        Values.spindexerConstants.spindexerPosition = Values.spindexerConstants.spindexerPurple1;
//                        break;
//                    case GPP:
//                        Values.spindexerConstants.sA = 400000;
//                        Values.spindexerConstants.spindexerPosition = Values.spindexerConstants.spindexerGreen;
//                        break;
//                }
                if (!follower.isBusy()) {
                    follower.followPath(scorePreload, true);
                    Values.flywheelConstants.flywheelVelocity = 1560;
                    pathTimer.resetTimer();

                    if (Values.motif!= Values.Motif.NONE) {
                        robot.limelight.stop();
                        nextPath();
                    }
                }

                break;

            case 1:
            case 18:
            case 35:

                    switch (Values.motif) {
                        case PPG:
                        case PGP:
                            Values.spindexerConstants.sA = 100000;
                            Values.spindexerConstants.spindexerPosition = Values.spindexerConstants.spindexerPurple1;
                            break;
                        case GPP:
                            Values.spindexerConstants.sA = 100000;
                            ;
                            Values.spindexerConstants.spindexerPosition = Values.spindexerConstants.spindexerGreen;
                            break;
                    }

                if (Math.abs(robot.spindexer.getCurrentPosition() - Values.spindexerConstants.spindexerPosition) < 200)
                    nextPath();
                break;

            case 2:
            case 10:
            case 6:
            case 19:
            case 23:
            case 27:
            case 36:
            case 40:
            case 44:
                robot.intake.setPower(0);
                transferBelt();
                if (pathTimer.getElapsedTimeSeconds() > .6) nextPath();
                break;

            case 3:
            case 11:
            case 7:
            case 20:
            case 24:
            case 28:
            case 37:
            case 41:
            case 45:
                if (Math.abs(Values.flywheelConstants.flywheelVelocity - robot.flywheel.getVelocity()) < 20
                        && !follower.isBusy()) {
                    transferKick();
                    pathTimer.resetTimer();
                    nextPath();
                }
                break;

            case 4:
            case 12:
            case 8:
            case 21:
            case 25:
            case 29:
            case 38:
            case 42:
            case 46:
                if (pathTimer.getElapsedTimeSeconds() > .5) {
                    transferStop();
                    nextPath();
                }
                break;

            case 5:
            case 22:
            case 39:
                switch (Values.motif) {
                    case PPG:
                        Values.spindexerConstants.sA = 400000;
                        Values.spindexerConstants.spindexerPosition = Values.spindexerConstants.spindexerPurple2;
                        break;
                    case PGP:
                        Values.spindexerConstants.sA = 400000;
                        Values.spindexerConstants.spindexerPosition = Values.spindexerConstants.spindexerGreen;
                        break;
                    case GPP:
                        Values.spindexerConstants.sA = 400000;
                        Values.spindexerConstants.spindexerPosition = Values.spindexerConstants.spindexerPurple1;
                        break;
                }
                if (Math.abs(robot.spindexer.getCurrentPosition() - Values.spindexerConstants.spindexerPosition) < 200)
                    nextPath();
                break;

            case 9:
            case 26:
            case 43:
                switch (Values.motif) {
                    case PPG:
                        Values.spindexerConstants.sA = 400000;
                        Values.spindexerConstants.spindexerPosition = Values.spindexerConstants.spindexerGreen;
                        break;
                    case PGP:
                    case GPP:
                        Values.spindexerConstants.sA = 100000;
                        Values.spindexerConstants.spindexerPosition = Values.spindexerConstants.spindexerPurple2;
                        break;
                }
                if (Math.abs(robot.spindexer.getCurrentPosition() - Values.spindexerConstants.spindexerPosition) < 400)
                    nextPath();
                break;


            case 13:
                Values.flywheelConstants.flywheelVelocity = 0;
                transferStop();
                follower.setMaxPower(1);
                if (!caseInitialized) {
                    Values.spindexerConstants.spindexerPosition =
                            Values.spindexerConstants.spindexerPurpleTransfer1;
                    follower.followPath(toFirstChain);
                    pathTimer.resetTimer();
                    caseInitialized = true;
                }

                if (Math.abs(robot.spindexer.getCurrentPosition() -
                        Values.spindexerConstants.spindexerPosition) < 200 && follower.getDriveError()<1) {
                    nextPath();
                    caseInitialized = false;
                }
                break;


            case 14:
                robot.intake.setPower(1);

                if (!caseInitialized) {
                    follower.followPath(grabFirstChain);

                    follower.setMaxPower(0.35);
                    pathTimer.resetTimer();
                    caseInitialized = true;
                }
                Values.spindexerConstants.sA = 400000;

                if (pathTimer.getElapsedTimeSeconds() > 2 &&
                        Math.abs(robot.spindexer.getCurrentPosition() -
                                Values.spindexerConstants.spindexerPosition) < 200) {


                    nextPath();
                    caseInitialized = false;
                }
                break;


            case 15:
                if (!caseInitialized) {
                    follower.setMaxPower(0.6);
                    follower.followPath(grabSecondChain);
                    Values.spindexerConstants.spindexerPosition =
                            Values.spindexerConstants.spindexerPurpleTransfer3;
                    pathTimer.resetTimer();
                    caseInitialized = true;
                }
                Values.spindexerConstants.sA = 400000;

                if (pathTimer.getElapsedTimeSeconds() > 2 &&
                        Math.abs(robot.spindexer.getCurrentPosition() -
                                Values.spindexerConstants.spindexerPosition) < 200) {


                    nextPath();
                    caseInitialized = false;
                }
                break;

            case 16:
                if (!caseInitialized) {
                    follower.setMaxPower(1);
                    follower.followPath(grabThirdChain);
                    Values.spindexerConstants.sA=250000;
                    Values.spindexerConstants.spindexerPosition =
                            Values.spindexerConstants.spindexerGreenTransfer;
                    pathTimer.resetTimer();
                    caseInitialized = true;
                }

                if (pathTimer.getElapsedTimeSeconds() > 1.5 &&
                        Math.abs(robot.spindexer.getCurrentPosition() -
                                Values.spindexerConstants.spindexerPosition) < 200) {

                    nextPath();
                    caseInitialized = false;
                }
                break;

            case 17:
                Values.flywheelConstants.flywheelVelocity = 1560;
                follower.setMaxPower(1);
                if(Values.team.equals("red")) {
                    Values.turretConstants.turretPosition = 860;
                }else{
                    Values.turretConstants.turretPosition=-860;
                }
                if (!caseInitialized) {
                    follower.followPath(scoreFirstChain);
                    caseInitialized = true;
                }

                if (!follower.isBusy()) {
                    nextPath();
                    caseInitialized = false;
                }
                break;
            case 30:
                Values.flywheelConstants.flywheelVelocity = 0;
                transferStop();
                follower.setMaxPower(1);
                Values.turretConstants.turretPosition=0;
                if (!caseInitialized) {
                    Values.spindexerConstants.spindexerPosition =
                            Values.spindexerConstants.spindexerPurpleTransfer1;
                    follower.followPath(toSecondChain);
                    pathTimer.resetTimer();
                    caseInitialized = true;
                }
                if (Math.abs(robot.spindexer.getCurrentPosition() -
                        Values.spindexerConstants.spindexerPosition) < 200 && follower.getDriveError()<1) {
                    nextPath();
                    caseInitialized = false;
                }
                break;
            case 31:
                robot.intake.setPower(1);

                if (!caseInitialized) {
                    follower.setMaxPower(0.35);
                    follower.followPath(grabFourthChain);


                    pathTimer.resetTimer();
                    caseInitialized = true;
                }
                Values.spindexerConstants.sA = 400000;

                if (pathTimer.getElapsedTimeSeconds() > 2.5 &&
                        Math.abs(robot.spindexer.getCurrentPosition() -
                                Values.spindexerConstants.spindexerPosition) < 200) {


                    nextPath();
                    caseInitialized = false;
                }
                break;
            case 32:
                if (!caseInitialized) {
                    follower.setMaxPower(0.5);
                    follower.followPath(grabFifthChain);
                    Values.spindexerConstants.spindexerPosition =
                            Values.spindexerConstants.spindexerPurpleTransfer3;
                    pathTimer.resetTimer();
                    caseInitialized = true;
                }
                Values.spindexerConstants.sA = 400000;

                if (pathTimer.getElapsedTimeSeconds() > 2.5 &&
                        Math.abs(robot.spindexer.getCurrentPosition() -
                                Values.spindexerConstants.spindexerPosition) < 200) {


                    nextPath();
                    caseInitialized = false;
                }
                break;
            case 33:
                if (!caseInitialized) {
                    follower.setMaxPower(1);
                    follower.followPath(grabSixthChain);
                    Values.spindexerConstants.sA=400000;
                    Values.spindexerConstants.spindexerPosition =
                            Values.spindexerConstants.spindexerGreenTransfer;
                    pathTimer.resetTimer();
                    caseInitialized = true;
                }

                if (pathTimer.getElapsedTimeSeconds() > 2 &&
                        Math.abs(robot.spindexer.getCurrentPosition() -
                                Values.spindexerConstants.spindexerPosition) < 200) {
                    Values.spindexerConstants.spindexerPosition=0;
                    caseInitialized = false;
                }
                break;
            case 34:
                Values.flywheelConstants.flywheelVelocity = 1600;
                follower.setMaxPower(1);

                if (!caseInitialized) {
                    follower.followPath(scoreSecondChain);
                    caseInitialized = true;
                }

                if (!follower.isBusy()) {
                    nextPath();
                    caseInitialized = false;
                }
                break;
            case 47:

                follower.followPath(leave,1,true);
                transferStop();
                Values.spindexerConstants.spindexerPosition=Values.spindexerConstants.spindexerStart;






        }
    }




//
//            case 1:
//                if (!follower.isBusy()) {
//                    follower.followPath(toFirstChain);
//                    setPathState(2);
//                }
//                break;
//
//            case 2:
//                if (!follower.isBusy()) {
//                    follower.followPath(grabFirstChain);
//                    setPathState(3);
//                }
//                break;
//
//            case 3:
//                if (!follower.isBusy()) {
//                    follower.followPath(grabSecondChain);
//                    setPathState(4);
//                }
//                break;
//
//            case 4:
//                if (!follower.isBusy()) {
//                    follower.followPath(grabThirdChain);
//                    setPathState(5);
//                }
//                break;
//
//            case 5:
//                if (!follower.isBusy()) {
//                    follower.followPath(scoreFirstChain);
//                    setPathState(6);
//                }
//                break;
//            case 6:
//                if (!follower.isBusy()){
//                    follower.followPath(toSecondChain);
//                    setPathState(7);
//                }
//                break;
//            case 7:
//                if (!follower.isBusy()){
//                    follower.followPath(grabFourthChain);
//                    setPathState(8);
//                }
//                break;
//            case 8:
//                if (!follower.isBusy()){
//                    follower.followPath(grabFifthChain);
//                    setPathState(9);
//                }
//                break;
//            case 9:
//                if (!follower.isBusy()){
//                    follower.followPath(grabSixthChain);
//                    setPathState(10);
//                }
//                break;
//            case 10:
//                if (!follower.isBusy()){
//                    follower.followPath(scoreSecondChain);
//                    setPathState(11);
//                }
//                break;
//            case 11:
//                if (!follower.isBusy()){
//                    follower.followPath(leave);
//                    setPathState(99);
//                }
//                break;

    public void nextPath(){
        pathState++;
        pathTimer.resetTimer();
    }
    public void setPathState(int p) {
        pathState = p;
        pathTimer.resetTimer();
        actionState=0;
    }

    public void nextAction() {
        actionState ++;
        actionTimer.resetTimer();
    }
    public Values.Motif getMotif(Limelight3A ll, Telemetry telemetry) {
        List<LLResultTypes.FiducialResult> result = ll.getLatestResult().getFiducialResults();
        if (!result.isEmpty()) {

            for (LLResultTypes.FiducialResult fiducial : result){
                int id = fiducial.getFiducialId();
                switch (id) {
                    case 21:
                        return Values.Motif.GPP;
                    case 22:
                        return Values.Motif.PGP;
                    case 23:
                        return Values.Motif.PPG;
                }
            }
        }
        telemetry.addData("tags",result);

        return Values.Motif.NONE;
    }
    public static Pose mirror(Pose p) {
        double x = 144 - p.getX();
        double y = p.getY();
        double h = p.getHeading();

        if (!Double.isNaN(h)) {
            h = Math.PI - h; // mirror the heading
        }

        return new Pose(x, y, h);
    }

}


