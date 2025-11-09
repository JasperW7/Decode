package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import com.arcrobotics.ftclib.controller.PIDFController;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.pedropathing.util.Timer;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * This is an example auto that showcases movement and control of two servos autonomously.
 * It is a 0+4 (Specimen + Sample) bucket auto. It scores a neutral preload and then pickups 3 samples from the ground and scores them before parking.
 * There are examples of different ways to build paths.
 * A path progression method has been created and can advance based on time, position, or other factors.
 *
 * @author Baron Henderson - 20077 The Indubitables
 * @version 2.0, 11/28/2024
 */

@Autonomous(name = "blue leave auton", group = "Auto")
public class LeaveBlue extends OpMode {
    private DcMotorEx AMotor,S1Motor,S2Motor;
    private Servo wrist,claw,rotation;
    private Limelight3A limelight;
    private Follower follower;
    private Timer pathTimer, actionTimer, opmodeTimer,loopTimer;

    public double wristPar = 0.1, wristPerp = 0.62, wristOuttake = 0.82;
    public double clawOpen =  0.3, clawClose = 0.74;
    public double rotationPos = 0.46;
    public double armDown = 25;
    public double armPar = 100, armUp = 890, slidePar = 100,slideUp = 1400;
    public int slideInterval = 15;
    public double outToRestBuffer = 600, restToOuttake = 1000;
    public boolean intaking = false;
    //  ARM PID
    PIDFController armPIDF = new PIDFController(0,0,0, 0);
    double armP = 0.03, armI = 0, armD = 0, armF = 0;
    double armPE = 0.01, armIE = 0, armDE = 0, armFE = 0.005;
    double armTarget = 0.0;

    //  SLIDES PID
    PIDFController slidePIDF = new PIDFController(0,0,0, 0);
    double slideP = 0.017, slideI = 0, slideD = 0.00018, slideF = 0;
    //    static double slidePE = 0.008, slideIE = 0, slideDE = 0.00018, slideFE = 0;
    double slideTarget = 0.0;
    double slidePower = 0.0;

    /** This is the variable where we store the state of our auto.
     * It is used by the pathUpdate method. */
    private int pathState,actionState;

    /* Create and Define Poses + Paths
     * Poses are built with three constructors: x, y, and heading (in Radians).
     * Pedro uses 0 - 144 for x and y, with 0, 0 being on the bottom left.
     * (For Into the Deep, this would be Blue Observation Zone (0,0) to Red Observation Zone (144,144).)
     * Even though Pedro uses a different coordinate system than RR, you can convert any roadrunner pose by adding +72 both the x and y.
     * This visualizer is very easy to use to find and create paths/pathchains/poses: <https://pedro-path-generator.vercel.app/>
     * Lets assume our robot is 18 by 18 inches
     * Lets assume the Robot is facing the human player and we want to score in the bucket */

    /** Start Pose of our robot */
    private final Pose startPose = new Pose(9, 111, Math.toRadians(270));
    private final Pose leave = new Pose(70,111,Math.toRadians(270));

    /** Scoring Pose of our robot. It is facing the submersible at a -45 degree (315 degree) angle. */

    /* These are our Paths and PathChains that we will define in buildPaths() */
    private Path park;
    private PathChain leaveRP,scorePreload,grabPickup1, grabPickup2, grabPickup3, scorePickup1, scorePickup2, scorePickup3;

    /** Build the paths for the auto (adds, for example, constant/linear headings while doing paths)
     * It is necessary to do this so that all the paths are built before the auto starts. **/
    public void buildPaths() {

        /* There are two major types of paths components: BezierCurves and BezierLines.
         *    * BezierCurves are curved, and require >= 3 points. There are the start and end points, and the control points.
         *    - Control points manipulate the curve between the start and end points.
         *    - A good visualizer for this is [this](https://pedro-path-generator.vercel.app/).
         *    * BezierLines are straight, and require 2 points. There are the start and end points.
         * Paths have can have heading interpolation: Constant, Linear, or Tangential
         *    * Linear heading interpolation:
         *    - Pedro will slowly change the heading of the robot from the startHeading to the endHeading over the course of the entire path.
         *    * Constant Heading Interpolation:
         *    - Pedro will maintain one heading throughout the entire path.
         *    * Tangential Heading Interpolation:
         *    - Pedro will follows the angle of the path such that the robot is always driving forward when it follows the path.
         * PathChains hold Path(s) within it and are able to hold their end point, meaning that they will holdPoint until another path is followed.
         * Here is a explanation of the difference between Paths and PathChains <https://pedropathing.com/commonissues/pathtopathchain.html> */

        leaveRP = follower.pathBuilder()
                .addPath(new BezierLine(startPose,leave))
                .setLinearHeadingInterpolation(startPose.getHeading(), leave.getHeading())
                .build();

    }




    public void autonomousPathUpdate() {

        switch (pathState) {
            case 0:
                follower.followPath(leaveRP);
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }

    public void setActionState(int aState){
        actionState = aState;
        actionTimer.resetTimer();
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() {

        // These loop the movements of the robot

    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        AMotor = hardwareMap.get(DcMotorEx.class,"AMotor");
        S1Motor = hardwareMap.get(DcMotorEx.class,"S1Motor");
        S2Motor = hardwareMap.get(DcMotorEx.class,"S2Motor");

        wrist = hardwareMap.get(Servo.class,"wrist");
        rotation = hardwareMap.get(Servo.class,"rotation");
        claw = hardwareMap.get(Servo.class,"claw");

        limelight = hardwareMap.get(Limelight3A.class,"limelight");
        pathTimer = new Timer();
        actionTimer = new Timer();
        opmodeTimer = new Timer();
        opmodeTimer.resetTimer();
        loopTimer = new Timer();
        loopTimer.resetTimer();




    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {}

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        setPathState(0);
        setActionState(0);
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
    }



}