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

    public static Pose startingPose = new Pose(15,112,Math.toRadians(0));
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

        pathTimer = new Timer();
        actionTimer = new Timer();
    }


    private void buildPaths() {

        // -------- BLUE --------
        scorePreloadBlue = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(15,112), new Pose(48,96)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(63.4))
                .build();

        toFirstChainBlue = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(48,96), new Pose(42,84)))
                .setLinearHeadingInterpolation(Math.toRadians(63.4), Math.toRadians(180))
                .build();

        grabFirstBlue = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(42,84), new Pose(36.5,84)))
                .setTangentHeadingInterpolation()
                .build();

        grabSecondBlue = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(36.5,84), new Pose(31.5,84)))
                .setTangentHeadingInterpolation()
                .build();

        grabThirdBlue = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(31.5,84), new Pose(26.5,84)))
                .setTangentHeadingInterpolation()
                .build();

        scoreFirstChainBlue = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(26.5,84), new Pose(48,96)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        toSecondChainBlue = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(48,96), new Pose(42,60)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        grabFourthBlue = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(42,60), new Pose(36.5,60)))
                .setTangentHeadingInterpolation()
                .build();

        grabFifthBlue = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(36.5,60), new Pose(31.5,60)))
                .setTangentHeadingInterpolation()
                .build();

        grabSixthBlue = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(31.5,60), new Pose(26.5,60)))
                .setTangentHeadingInterpolation()
                .build();

        scoreSecondChainBlue = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(26.5,60), new Pose(48,96)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(180))
                .build();

        leaveBlue = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(48,96), new Pose(20,72)))
                .setTangentHeadingInterpolation()
                .build();



        // -------- RED --------
        scorePreloadRed = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(129,112),new Pose(115.3,117.4), new Pose(104.6,107)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(116.6))
                .build();

        toFirstChainRed = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(104.6,107),new Pose(88.7,98), new Pose(100,86)))
                .setLinearHeadingInterpolation(Math.toRadians(116.6), Math.toRadians(0))
                .build();

        grabFirstRed = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(100,86), new Pose(107,86)))
                .setTangentHeadingInterpolation()
                .build();

        grabSecondRed = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(107,86), new Pose(113,86)))
                .setTangentHeadingInterpolation()
                .build();

        grabThirdRed = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(113,86), new Pose(127,86)))
                .setTangentHeadingInterpolation()
                .build();

        scoreFirstChainRed = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(127,86), new Pose(96,96)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(260))
                .build();

        toSecondChainRed = follower.pathBuilder()
                .addPath(new BezierCurve(new Pose(96,96), new Pose(76.6,80), new Pose(100,64)))
                .setLinearHeadingInterpolation(Math.toRadians(260), Math.toRadians(0))
                .build();

        grabFourthRed = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(100,64), new Pose(107,64)))
                .setTangentHeadingInterpolation()
                .build();

        grabFifthRed = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(107,64), new Pose(113,64)))
                .setTangentHeadingInterpolation()
                .build();

        grabSixthRed = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(113,64), new Pose(120,64)))
                .setTangentHeadingInterpolation()
                .build();

        scoreSecondChainRed = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(120,64), new Pose(96,96)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        leaveRed = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(96,96), new Pose(124,72)))
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
        methods.positionPID(robot.turret,methods.turretAutoTrack(follower.getPose()),"turret");
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

                follower.setMaxPower(1);
                Values.motif=getMotif(robot.limelight,telemetry);
                switch (Values.motif) {
                    case PPG:
                    case PGP:
                        //values.spindexerconstants.sa = 200000;
                        Values.spindexerConstants.spindexerPosition = Values.spindexerConstants.spindexerPurple1;
                        break;
                    case GPP:
                        //values.spindexerconstants.sa = 200000;
                        Values.spindexerConstants.spindexerPosition = Values.spindexerConstants.spindexerGreen;
                        break;
                }
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
                        //values.spindexerconstants.sa = 200000;
                        Values.spindexerConstants.spindexerPosition = Values.spindexerConstants.spindexerPurple1;
                        break;
                    case GPP:
                        //values.spindexerconstants.sa = 200000;
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
                if (pathTimer.getElapsedTimeSeconds() > .3) nextPath();
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
                if (Math.abs(Values.flywheelConstants.flywheelVelocity - robot.flywheel.getVelocity()) < 50
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
                if (pathTimer.getElapsedTimeSeconds() > .2) {
                    transferBelt();
                    nextPath();
                }
                break;

            case 5:
            case 22:
            case 39:
                switch (Values.motif) {
                    case PPG:
                        //values.spindexerconstants.sa = 200000;
                        Values.spindexerConstants.spindexerPosition = Values.spindexerConstants.spindexerPurple2;
                        break;
                    case PGP:
                        //values.spindexerconstants.sa = 200000;
                        Values.spindexerConstants.spindexerPosition = Values.spindexerConstants.spindexerGreen;
                        break;
                    case GPP:
                        //values.spindexerconstants.sa = 200000;
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
                        //values.spindexerconstants.sa = 200000;
                        Values.spindexerConstants.spindexerPosition = Values.spindexerConstants.spindexerGreen;
                        break;
                    case PGP:
                    case GPP:
                        //values.spindexerconstants.sa = 200000;
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
                            Values.spindexerConstants.spindexerPurpleTransfer3;
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
                //values.spindexerconstants.sa=200000;

                if (pathTimer.getElapsedTimeSeconds() > 1.5 &&
                        Math.abs(robot.spindexer.getCurrentPosition() -
                                Values.spindexerConstants.spindexerPosition) < 200) {

                    Values.spindexerConstants.spindexerPosition =
                            Values.spindexerConstants.spindexerPurpleTransfer1;
                    nextPath();
                    caseInitialized = false;
                }
                break;


            case 15:
                if (!caseInitialized) {
                    follower.setMaxPower(0.35);
                    follower.followPath(grabSecondChain);
                    pathTimer.resetTimer();
                    caseInitialized = true;
                }
                //values.spindexerconstants.sa=200000;

                if (pathTimer.getElapsedTimeSeconds() > 1.5 &&
                        Math.abs(robot.spindexer.getCurrentPosition() -
                                Values.spindexerConstants.spindexerPosition) < 200) {

                    Values.spindexerConstants.spindexerPosition =
                            Values.spindexerConstants.spindexerGreenTransfer;
                    nextPath();
                    caseInitialized = false;
                }
                break;

            case 16:
                if (!caseInitialized) {
                    follower.setMaxPower(0.35);
                    follower.followPath(grabThirdChain);
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
                Values.flywheelConstants.flywheelVelocity = 1600;
                follower.setMaxPower(1);

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
                if (!caseInitialized) {
                    Values.spindexerConstants.spindexerPosition =
                            Values.spindexerConstants.spindexerPurpleTransfer3;
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
                    follower.followPath(grabFourthChain);

                    follower.setMaxPower(0.35);
                    pathTimer.resetTimer();
                    caseInitialized = true;
                }
                //values.spindexerconstants.sa=200000;

                if (pathTimer.getElapsedTimeSeconds() > 1.5 &&
                        Math.abs(robot.spindexer.getCurrentPosition() -
                                Values.spindexerConstants.spindexerPosition) < 200) {

                    Values.spindexerConstants.spindexerPosition =
                            Values.spindexerConstants.spindexerGreenTransfer;
                    nextPath();
                    caseInitialized = false;
                }
                break;
            case 32:
                if (!caseInitialized) {
                    follower.setMaxPower(0.35);
                    follower.followPath(grabFifthChain);
                    pathTimer.resetTimer();
                    caseInitialized = true;
                }
                //values.spindexerconstants.sa=200000;

                if (pathTimer.getElapsedTimeSeconds() > 1.5 &&
                        Math.abs(robot.spindexer.getCurrentPosition() -
                                Values.spindexerConstants.spindexerPosition) < 200) {

                    Values.spindexerConstants.spindexerPosition =
                            Values.spindexerConstants.spindexerPurpleTransfer1;
                    nextPath();
                    caseInitialized = false;
                }
                break;
            case 33:
                if (!caseInitialized) {
                    follower.setMaxPower(0.35);
                    follower.followPath(grabSixthChain);
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
}


