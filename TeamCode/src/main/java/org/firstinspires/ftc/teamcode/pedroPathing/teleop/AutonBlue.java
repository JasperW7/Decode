package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import com.pedropathing.follower.Follower;
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

@Autonomous(name = "blue leave auton", group = "Auto")
public class AutonBlue extends OpMode {

    private Hardware robot;
    private Follower follower;

    // -------------------------
    // ALL PATHS (COMBINED CLASS)
    // -------------------------
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

    // Mapped paths used during auton (assigned on start)
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

        robot = new Hardware(hardwareMap);
        robot.init();
        robot.limelight.start();
        // BUILD PATHS HERE (formerly in Paths class)
        buildPaths();

        pathTimer = new Timer();
        actionTimer = new Timer();
    }

    // ----------------------------
    // BUILD BLUE + RED PATHS HERE
    // ----------------------------
    private void buildPaths() {

        // -------- BLUE --------
        scorePreloadBlue = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(15,112), new Pose(48,96)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(63.4))
                .build();

        toFirstChainBlue = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(48,96), new Pose(42,84)))
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
                .addPath(new BezierLine(new Pose(129,112), new Pose(96,96)))
                .setLinearHeadingInterpolation(Math.toRadians(180), Math.toRadians(116.6))
                .build();

        toFirstChainRed = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(96,96), new Pose(102,84)))
                .setLinearHeadingInterpolation(Math.toRadians(116.6), Math.toRadians(0))
                .build();

        grabFirstRed = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(102,84), new Pose(107.5,84)))
                .setTangentHeadingInterpolation()
                .build();

        grabSecondRed = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(107.5,84), new Pose(112.5,84)))
                .setTangentHeadingInterpolation()
                .build();

        grabThirdRed = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(112.5,84), new Pose(117.5,84)))
                .setTangentHeadingInterpolation()
                .build();

        scoreFirstChainRed = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(117.5,84), new Pose(96,96)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        toSecondChainRed = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(96,96), new Pose(102,60)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        grabFourthRed = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(102,60), new Pose(107.5,60)))
                .setTangentHeadingInterpolation()
                .build();

        grabFifthRed = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(107.5,60), new Pose(112.5,60)))
                .setTangentHeadingInterpolation()
                .build();

        grabSixthRed = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(112.5,60), new Pose(117.5,60)))
                .setTangentHeadingInterpolation()
                .build();

        scoreSecondChainRed = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(117.5,60), new Pose(96,96)))
                .setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                .build();

        leaveRed = follower.pathBuilder()
                .addPath(new BezierLine(new Pose(96,96), new Pose(124,72)))
                .setTangentHeadingInterpolation()
                .build();

    }

    // ----------------------------
    // SELECT TEAM
    // ----------------------------
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

    }

    // ----------------------------
    // START AUTON
    // ----------------------------
    @Override
    public void start() {

        follower.setStartingPose(startingPose);

        // Map paths depending on alliance
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
        telemetry.addData("state",pathState);
        telemetry.addData("position", follower.getPose());
        telemetry.addData("motif",Values.motif);
        telemetry.addData("fiducials", getMotif(robot.limelight,telemetry));
        methods.positionPID(robot.turret,methods.turretAutoTrack(follower.getPose()),"turret");
        methods.velocityPID(robot.flywheel,Values.flywheelConstants.flywheelVelocity,"flywheel");
        methods.positionPID(robot.spindexer,Values.spindexerConstants.spindexerPosition, "spindexer");
        telemetry.update();
    }

    public boolean intakeGreen(){
        Values.spindexerConstants.spindexerPosition=Values.spindexerConstants.spindexerGreenTransfer;
        return Math.abs(robot.spindexer.getCurrentPosition() - Values.spindexerConstants.spindexerPosition) < 200;
    }
    public boolean intakePurple1(){
        Values.spindexerConstants.spindexerPosition=Values.spindexerConstants.spindexerPurpleTransfer1;
        return Math.abs(robot.spindexer.getCurrentPosition() - Values.spindexerConstants.spindexerPosition) < 200;
    }
    public boolean intakePurple2(){
        Values.spindexerConstants.spindexerPosition=Values.spindexerConstants.spindexerPurpleTransfer3;
        return Math.abs(robot.spindexer.getCurrentPosition() - Values.spindexerConstants.spindexerPosition) < 200;
    }
    public boolean transferGreen(){
        Values.spindexerConstants.spindexerPosition=Values.spindexerConstants.spindexerGreen;
        return Math.abs(robot.spindexer.getCurrentPosition() - Values.spindexerConstants.spindexerPosition) < 200;
    }
    public boolean transferPurple1(){
        Values.spindexerConstants.spindexerPosition=Values.spindexerConstants.spindexerPurple1;
        return Math.abs(robot.spindexer.getCurrentPosition() - Values.spindexerConstants.spindexerPosition) < 200;
    }
    public boolean transferPurple2(){
        Values.spindexerConstants.spindexerPosition=Values.spindexerConstants.spindexerPurple2;
        return Math.abs(robot.spindexer.getCurrentPosition() - Values.spindexerConstants.spindexerPosition) < 200;
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

    public boolean flywheel(){
        Values.flywheelConstants.flywheelVelocity=1600;
        return Math.abs(Values.flywheelConstants.flywheelVelocity - robot.flywheel.getVelocity()) < 50;
    }

    public boolean scoring() {
        String[] sequence;
        switch (Values.motif) {
            case "PPG":
                sequence = new String[]{"Purple1","Belt","Flywheel","Kick","Purple2","Belt","Flywheel","Kick","Green","Belt","Flywheel","Kick"};
                break;
            case "PGP":
                sequence = new String[]{"Purple1","Belt","Flywheel","Kick","Green","Belt","Flywheel","Kick","Purple2","Belt","Flywheel","Kick"};
                break;
            case "GPP":
                sequence = new String[]{"Green","Belt","Flywheel","Kick","Purple1","Belt","Flywheel","Kick","Purple2","Belt","Flywheel","Kick"};
                break;
            default:
                sequence = new String[0];
        }

        if (actionState < sequence.length) {
            String current = sequence[actionState];

            switch (current) {
                case "Purple1":
                    if (transferPurple1()) nextAction();
                    break;
                case "Purple2":
                    if (transferPurple2()) nextAction();
                    break;
                case "Green":
                    if (transferGreen()) nextAction();
                    break;
                case "Belt":
                    transferBelt();
                    if (actionTimer.getElapsedTimeSeconds() > 0.8) nextAction();
                    break;
                case "Flywheel":
                    if (flywheel()) nextAction();
                    break;
                case "Kick":
                    // Only kick if the path is finished
                    if (!follower.isBusy()) {
                        transferKick();
                        if (actionTimer.getElapsedTimeSeconds() > 0.5) {
                            transferStop();
                            nextAction();
                        }
                    }
                    break;
            }
        }

        // Return true only when the sequence is fully complete
        return actionState == sequence.length;
    }




    // ----------------------------
    // MAIN PATH STATE MACHINE
    // ----------------------------


    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:
                if (!follower.isBusy()) {
                    follower.followPath(scorePreload, true);
                    Values.flywheelConstants.flywheelVelocity = 1600; // spin up flywheel
                    pathTimer.resetTimer();
                }


                if (!follower.isBusy()) {
                    nextPath();
                    Values.motif=getMotif(robot.limelight,telemetry);
                };
                break;

            case 1:
                Values.motif=getMotif(robot.limelight,telemetry);
                switch (Values.motif) {
                    case "PPG":
                    case "PGP":
                        Values.spindexerConstants.spindexerPosition = Values.spindexerConstants.spindexerPurpleTransfer1;
                        break;
                    case "GPP":
                        Values.spindexerConstants.spindexerPosition = Values.spindexerConstants.spindexerGreenTransfer;
                        break;
                }
                if (Math.abs(robot.spindexer.getCurrentPosition() - Values.spindexerConstants.spindexerPosition) < 200)
                    nextPath();
                break;

            case 2:

            case 10:

            case 6:
                transferBelt();
                if (pathTimer.getElapsedTimeSeconds() > 0.8) nextPath();
                break;

            case 3:

            case 11:

            case 7:
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
                if (pathTimer.getElapsedTimeSeconds() > 0.5) {
                    transferStop();
                    nextPath();
                }
                break;

            case 5:
                switch (Values.motif) {
                    case "PPG":
                        Values.spindexerConstants.spindexerPosition = Values.spindexerConstants.spindexerPurpleTransfer3;
                        break;
                    case "PGP":
                        Values.spindexerConstants.spindexerPosition = Values.spindexerConstants.spindexerGreenTransfer;
                        break;
                    case "GPP":
                        Values.spindexerConstants.spindexerPosition = Values.spindexerConstants.spindexerPurpleTransfer1;
                        break;
                }
                if (Math.abs(robot.spindexer.getCurrentPosition() - Values.spindexerConstants.spindexerPosition) < 200)
                    nextPath();
                break;

            case 9:
                switch (Values.motif) {
                    case "PPG":
                        Values.spindexerConstants.spindexerPosition = Values.spindexerConstants.spindexerGreenTransfer;
                        break;
                    case "PGP":
                    case "GPP":
                        Values.spindexerConstants.spindexerPosition = Values.spindexerConstants.spindexerPurpleTransfer3;
                        break;
                }
                if (Math.abs(robot.spindexer.getCurrentPosition() - Values.spindexerConstants.spindexerPosition) < 200)
                    nextPath();
                break;

            case 13:
                if (!follower.isBusy()) {
                    follower.followPath(toFirstChain);
                    nextPath();
                }
                break;

            // Add additional paths/states as needed after scoring
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
    public String getMotif(Limelight3A ll, Telemetry telemetry) {
        List<LLResultTypes.FiducialResult> result = ll.getLatestResult().getFiducialResults();
        if (!result.isEmpty()) {

            for (LLResultTypes.FiducialResult fiducial : result){
                int id = fiducial.getFiducialId();
                switch (id) {
                    case 21:
                        return "GPP";
                    case 22:
                        return "PGP";
                    case 23:
                        return "PPG";
                    default:
                        return Integer.toString(id);
                }
            }
        }
        telemetry.addData("tags",result);

        return "";
    }
}
