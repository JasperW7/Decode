package org.firstinspires.ftc.teamcode.pedroPathing.teleop;
import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.hardware.ServoEx;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;

import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.HeadingInterpolator;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.pedroPathing.teleop.Values;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Arrays;
import java.util.function.Supplier;

@Configurable
@TeleOp
public class Teleop extends OpMode {
    private Hardware robot;

    private Follower follower;
    private ElapsedTime elapsedTime;


    public static Pose startingPose = new Pose(15,112,Math.toRadians(0));
    private boolean automatedDrive;
    private Supplier<PathChain> pathChain;
    private TelemetryManager telemetryM;
    private Methods methods = new Methods();
    private FtcDashboard dashboard = FtcDashboard.getInstance();
//    private LoopTimer lt = new LoopTimer();



    @Override
    public void init() {
        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(startingPose == null ? new Pose() : startingPose);
        follower.update();
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        robot = new Hardware(hardwareMap);
        robot.init();
        pathChain = () -> follower.pathBuilder() //Lazy Curve Generation
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(38.6, 33.1))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(180), 0.8))
                .build();
        elapsedTime = new ElapsedTime();
        elapsedTime.reset();
        robot.limelight.start();
    }

    @Override
    public void init_loop(){
        /** This method is called continuously after Init while waiting for "play". **/
        Values.spindexerConstants.index=0;

        Values.spindexerConstants.spindexerPosition = Values.spindexerConstants.indexer[Values.spindexerConstants.index];
        methods.positionPID(robot.turret,methods.turretAutoTrack(follower.getPose()),"turret");
        methods.positionPID(robot.spindexer,Values.spindexerConstants.spindexerStart,"spindexer");
        if (gamepad1.aWasPressed()){
            Values.team="blue";
            startingPose = new Pose(15,112,Math.toRadians(0));
        }else if (gamepad1.bWasPressed()){
            Values.team="red";
            startingPose = new Pose(129,112,Math.toRadians(180));
        }
        if (gamepad1.leftStickButtonWasPressed()){
            Values.drivers=!Values.drivers;
        }
        telemetry.addData("RESET","SPINDEXER & TURRET POSITION");
        telemetry.addData("Team: ", Values.team);
        telemetry.addData("drivers", (Values.drivers)?"1" : "2");
        telemetry.addData("position",follower.getPose());
        telemetry.update();

    }

    @Override
    public void start() {
        follower.setPose(startingPose);
        //The parameter controls whether the Follower should use break mode on the motors (using it is recommended).
        //In order to use float mode, add .useBrakeModeInTeleOp(true); to your Drivetrain Constants in Constant.java (for Mecanum)
        //If you don't pass anything in, it uses the default (false)
        follower.startTeleopDrive();

    }

    @Override
    public void loop() {
        //Call this once per loop
        follower.update();
        telemetryM.update();

        if (!automatedDrive) {
            if(!Values.drivers) {
                follower.setTeleOpDrive(
                        -gamepad2.left_stick_y,
                        -gamepad2.left_stick_x,
                        -gamepad2.right_stick_x,
                        true);
            }else{
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        true);
            }
        }

        if (gamepad1.leftBumperWasPressed() && Values.engaged!=2){
            if (Values.mode == Values.Mode.OUTTAKING){
                Values.purpleCount=0;
                Values.greenCount=0;
                Values.mode = Values.Mode.INTAKING;
                Values.init=true;
            }else if (Values.mode == Values.Mode.ENDGAME){
                Values.mode = Values.Mode.INTAKING;
                Values.init=true;
            }
        }
        if (gamepad1.rightBumperWasPressed()){
            if (Values.mode == Values.Mode.INTAKING){
                Values.mode = Values.Mode.OUTTAKING;
                Values.init=true;
            }else if (Values.mode == Values.Mode.OUTTAKING){
                Values.mode = Values.Mode.ENDGAME;
                Values.init=true;
            }else if (Values.mode == Values.Mode.ENDGAME){
                Values.mode = Values.Mode.OUTTAKING;
                Values.init=true;

            }
        }


        switch (Values.mode) {
            case INTAKING:
                if (Values.init) {
                    Values.spindexerConstants.index = 3;
                    Values.flywheelConstants.flywheelVelocity = 0;
                    Values.init = false;
                    Values.spindexerConstants.sA=200000;
                    Values.reversingIntake = false;
                    Values.purpleBallProcessed = false;
                    Values.greenBallProcessed = false;
                    Values.engaged = 0;
                    robot.transfer.setPosition(Values.transferBeltStop);
                    robot.transferEngage.setPosition(Values.transferDisengage);
                    Values.waitingOnSpindex = false;
                    Values.lastDetectedColor = Methods.DetectedColor.UNKNOWN;
                    Values.lastColorFrames = 0;
                }

                Values.turretConstants.turretPosition = Values.turretConstants.turretStart;

                double distance = robot.colorSensor.getDistance(DistanceUnit.CM);
                Methods.DetectedColor rawColor = methods.getDetectedColor(robot.colorSensor, telemetry);

                if (rawColor == Values.lastDetectedColor && rawColor != Methods.DetectedColor.UNKNOWN) {
                    Values.lastColorFrames++;
                } else {
                    Values.lastColorFrames = 0;
                    Values.lastDetectedColor = rawColor;
                }

                Methods.DetectedColor color = (Values.lastColorFrames >= 2) ? rawColor : Methods.DetectedColor.UNKNOWN;

                telemetry.addData("rawColor", rawColor);
                telemetry.addData("stableColor", color);

                if (gamepad1.leftBumperWasPressed()) {
                    if (Values.lastDetectedColor == Methods.DetectedColor.GREEN) {
                        Values.greenCount = Math.max(0, Values.greenCount - 1);
                    } else if (Values.lastDetectedColor == Methods.DetectedColor.PURPLE) {
                        Values.purpleCount = Math.max(0, Values.purpleCount - 1);
                    }

                    Values.reversingIntake = false;
                    Values.waitingOnSpindex = false;

                    Values.greenBallProcessed = false;
                    Values.purpleBallProcessed = false;

                    robot.led.setPosition(0);
                }

                if (Values.endgame) {
                    if (Values.purpleCount >= 3) {
                        Values.reversingColor = Methods.DetectedColor.PURPLE;
                        Values.reversingIntake = true;
                    } else if (Values.greenCount >= 2) {
                        Values.reversingColor = Methods.DetectedColor.GREEN;
                        Values.reversingIntake = true;
                    }
                } else {
                    if ((Values.purpleCount + Values.greenCount) > 3) {
                        Values.reversingColor = Methods.DetectedColor.UNKNOWN;
                        Values.reversingIntake = true;
                    }
                }

                if (Values.reversingIntake) {
                    robot.intake.setPower(-1);
                    robot.led.setPosition(.277);

                    boolean ballGone = (color == Methods.DetectedColor.UNKNOWN) && distance > 4.0;

                    if (ballGone) {
                        if (Values.reversingColor == Methods.DetectedColor.PURPLE) {
                            Values.purpleCount = Math.max(0, Values.purpleCount - 1);
                        } else if (Values.reversingColor == Methods.DetectedColor.GREEN) {
                            Values.greenCount = Math.max(0, Values.greenCount - 1);
                        } else {
                            if (Values.purpleCount + Values.greenCount > 0) {
                                if (Values.purpleCount > 0) Values.purpleCount--;
                                else if (Values.greenCount > 0) Values.greenCount--;
                            }
                        }

                        Values.reversingIntake = false;
                        Values.reversingColor = Methods.DetectedColor.UNKNOWN;
                        Values.purpleBallProcessed = false;
                        Values.greenBallProcessed = false;

                        robot.led.setPosition(0);
                    }

                    break;
                }

                if (Values.waitingOnSpindex) {
                    robot.intake.setPower(0);
                    int error = (int) Math.abs(
                            Values.spindexerConstants.spindexerPosition - robot.spindexer.getCurrentPosition()
                    );

                    if (error < 200)
                        Values.waitingOnSpindex = false;

                    break;
                }


                if (Values.endgame) {
                    robot.intake.setPower(1);
                    if (color == Methods.DetectedColor.PURPLE && !Values.purpleBallProcessed) {
                        robot.led.setPosition(.722);

                        if (Values.purpleCount == 0) Values.spindexerConstants.index = 4;
                        else if (Values.purpleCount == 1) Values.spindexerConstants.index = 5;

                        Values.purpleCount++;
                        Values.purpleBallProcessed = true;

                        Values.waitingOnSpindex = true;
                        break;
                    }

                    if (color == Methods.DetectedColor.GREEN && !Values.greenBallProcessed) {
                        robot.led.setPosition(.444);

                        if (Values.greenCount < 2) {
                            Values.spindexerConstants.sA=100000;
                            Values.spindexerConstants.index = 3;
                            Values.greenCount++;
                        }

                        Values.greenBallProcessed = true;

                        Values.waitingOnSpindex = true;
                        break;
                    }
                } else {
                    robot.led.setPosition(.611);
                    Values.purpleBallProcessed = false;
                    Values.greenBallProcessed = false;

                    if (gamepad1.aWasPressed()){
                        if (Values.spindexerConstants.index ==3){
                            Values.spindexerConstants.sA=200000;
                            Values.spindexerConstants.index=4;
                            methods.resetProfiledPID(Values.spindexerConstants.spindexerPIDF, robot.spindexer);
                        }else if (Values.spindexerConstants.index==4){
                            Values.spindexerConstants.index=5;
                            Values.spindexerConstants.sA=400000;
                            methods.resetProfiledPID(Values.spindexerConstants.spindexerPIDF, robot.spindexer);
                        }else if (Values.spindexerConstants.index==5){
                            Values.spindexerConstants.sA=400000;
                            Values.spindexerConstants.index=3;
                            methods.resetProfiledPID(Values.spindexerConstants.spindexerPIDF, robot.spindexer);
                        }
                    }

                    if (gamepad1.left_bumper){
                        robot.intake.setPower(.8);
                    }else{
                        robot.intake.setPower(0);
                    }
                }


                if (distance > 5.0 || color == Methods.DetectedColor.UNKNOWN) {
                    Values.purpleBallProcessed = false;
                    Values.greenBallProcessed = false;
                    robot.led.setPosition(0);
                }

                break;





            case OUTTAKING:
                if (Values.init){
                    Values.spindexerConstants.sA=200000;
                    Values.engaged = 0;
                    Values.spindexerConstants.index=0;
                    Values.init=false;
                }
                //methods.velocityPID(robot.flywheel,methods.calculateVelocity(robot.limelight),"flywheel");
                robot.intake.setPower(0);

                Values.turretConstants.turretPosition = methods.turretAutoTrack(follower.getPose());
                Values.flywheelConstants.flywheelVelocity=methods.interpolateVelocity(methods.getDistance(follower));
                if (methods.getDistance(follower)>84){
                    robot.led.setPosition(.277);
                }else if (Math.abs(robot.flywheel.getVelocity()-Values.flywheelConstants.flywheelVelocity)<90){
                    robot.led.setPosition(0.444);
                }else {
                    robot.led.setPosition(0);
                }
                if (gamepad1.aWasPressed() && Arrays.asList(new Integer[]{0,1,2}).contains(Values.spindexerConstants.index)){
                    switch (Values.engaged){
                        case 0:
                            Values.engaged = 1;
                            robot.transfer.setPosition(Values.transferBeltStart);
                            robot.transferEngage.setPosition(Values.transferEngage);
                            break;
                        case 1:
                            Values.engaged = 2;
                            robot.transfer.setPosition(Values.transferBeltMid);
                            robot.transferEngage.setPosition(Values.transferKick);
                            break;
                        case 2:
                            Values.engaged = 0;
                            robot.transfer.setPosition(Values.transferBeltStop);
                            robot.transferEngage.setPosition(Values.transferDisengage);
                    }
                }
                if (gamepad1.dpadRightWasPressed()){
                    Values.engaged=0;
                    if (Values.spindexerConstants.index==0){
                        Values.spindexerConstants.sA=400000;
                        Values.spindexerConstants.index=1;
                        methods.resetProfiledPID(Values.spindexerConstants.spindexerPIDF, robot.spindexer);
                    }else if (Values.spindexerConstants.index==1){
                        Values.spindexerConstants.sA=400000;
                        Values.spindexerConstants.index=2;
                        methods.resetProfiledPID(Values.spindexerConstants.spindexerPIDF, robot.spindexer);
                    }else if (Values.spindexerConstants.index==2){
                        Values.spindexerConstants.sA=200000;
                        Values.spindexerConstants.index=0;
                        methods.resetProfiledPID(Values.spindexerConstants.spindexerPIDF, robot.spindexer);
                    }
                }
                if (gamepad1.bWasPressed() && Values.engaged==1){
                    Values.engaged = 0;
                    robot.transfer.setPosition(Values.transferBeltStop);
                    robot.transferEngage.setPosition(Values.transferDisengage);
                }

                break;
            case ENDGAME:
                Values.flywheelConstants.flywheelVelocity=0;
                robot.intake.setPower(0);
                Values.engaged = 0;
                robot.transfer.setPosition(Values.transferBeltStop);
                robot.transferEngage.setPosition(Values.transferDisengage);


        }



//
//        if (gamepad1.dpad_left || gamepad1.dpad_right){
//            Values.spindexerConstants.override = true;
//        }
//        if (Values.spindexerConstants.override) {
//            if (gamepad1.b || gamepad1.x) {
//                Values.spindexerConstants.override = false;
//            }
//            Values.spindexerConstants.spindexerPosition += (gamepad1.dpad_right) ? 20 : 0;
//            Values.spindexerConstants.spindexerPosition -= (gamepad1.dpad_left) ? 20 : 0;
//        }else{
//            Values.spindexerConstants.index += (gamepad1.bWasPressed()) ? 1 : 0;
//            Values.spindexerConstants.index += (gamepad1.xWasPressed()) ? 5 : 0;
//            Values.spindexerConstants.index %= 6;
//            Values.spindexerConstants.spindexerPosition = Values.spindexerConstants.indexer[Values.spindexerConstants.index];
//
//        }

        methods.relocalize(robot.limelight,follower,telemetry);

        if (gamepad1.yWasPressed()){
            Values.endgame = !Values.endgame;
        }



        Values.spindexerConstants.spindexerPosition = Values.spindexerConstants.indexer[Values.spindexerConstants.index];
        methods.velocityPID(robot.flywheel,Values.flywheelConstants.flywheelVelocity, "flywheel");
        methods.positionPID(robot.turret,Values.turretConstants.turretPosition,"turret");
        methods.positionPID(robot.spindexer,Values.spindexerConstants.spindexerPosition,"spindexer");


//
//        //Automated PathFollowing
//        if (gamepad1.aWasPressed()) {
//            follower.followPath(pathChain.get());
//            automatedDrive = true;
//        }
//
//        //Stop automated following if the follower is done
//        if (automatedDrive && (gamepad1.bWasPressed() || ! follower.isBusy())) {
//            follower.startTeleopDrive();
//            automatedDrive = false;
//        }
//        lt.update();
        telemetry.addData("mode", Values.mode);
        telemetry.addData("index",Values.spindexerConstants.index);
        telemetry.addData("flywheel",robot.flywheel.getVelocity());
        telemetry.addData("flywheel power", robot.flywheel.getPower());
        telemetry.addData("flywheel target",Values.flywheelConstants.flywheelVelocity);
        telemetry.addData("transfer", Values.transferEngage);
        telemetry.addData("dist", methods.getDistance(follower));
        telemetry.addData("sorting",Values.endgame);
        telemetry.addData("team",Values.team);
        telemetry.addData("position", follower.getPose());
        telemetry.addData("loop time",elapsedTime.milliseconds());
        telemetry.addData("automatedDrive", automatedDrive);
        telemetry.update();
        elapsedTime.reset();
    }

    @Override
    public void stop(){
        Values.spindexerConstants.spindexerPosition=Values.spindexerConstants.spindexerStart;
    }
}
