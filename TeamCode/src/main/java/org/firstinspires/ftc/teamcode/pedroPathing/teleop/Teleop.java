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
import org.firstinspires.ftc.teamcode.pedroPathing.teleop.Values;

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.Arrays;
import java.util.function.Supplier;

@Configurable
@TeleOp
public class Teleop extends OpMode {
    private Hardware robot;

    private Follower follower;
    public static Pose startingPose = new Pose(33.4,134.5,Math.toRadians(90));
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
        }else if (gamepad1.bWasPressed()){
            Values.team="red";
        }
        telemetry.addData("RESET","SPINDEXER & TURRET POSITION");
        telemetry.addData("Team: ", Values.team);

    }

    @Override
    public void start() {
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
            //Make the last parameter false for field-centric
            //In case the drivers want to use a "slowMode" you can scale the vectors

            //This is the normal version to use in the TeleOp
            if (Values.mode==Values.Mode.ENDGAME){
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y*0.5,
                        -gamepad1.left_stick_x*0.5,
                        -gamepad1.right_stick_x*0.5,
                        true // Robot Centric
                );
            }else {
                follower.setTeleOpDrive(
                        -gamepad1.left_stick_y,
                        -gamepad1.left_stick_x,
                        -gamepad1.right_stick_x,
                        true // Robot Centric
                );
            }
        }

        if (gamepad1.leftBumperWasPressed()){
            if (Values.mode == Values.Mode.OUTTAKING){
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


        switch (Values.mode){
            case INTAKING:
                if (Values.init){
                    Values.spindexerConstants.index=3;
                    Values.flywheelConstants.flywheelVelocity=0;
                    Values.init=false;
                }
                Values.turretConstants.turretPosition = Values.turretConstants.turretStart;



                robot.intake.setPower(1);
                robot.transferEngage.setPosition(Values.transferDisengage);
                robot.transfer.setPosition(Values.transferBeltStop);


//
//                Methods.DetectedColor color = methods.getDetectedColor(robot.colorSensor, telemetry);
//
//                if (color != Methods.DetectedColor.UNKNOWN && color != robot.lastColor) {
//                    robot.lastColor = color;
//
//                    if (Values.mode == Values.Mode.INTAKING && color != Methods.DetectedColor.GREEN) {
//                        robot.led.setPosition(0.722);
//                    } else if (Values.mode == Values.Mode.INTAKING) {
//                        robot.led.setPosition(0.444);
//                    } else {
//                        robot.led.setPosition(0);
//                    }
//                }



                break;
            case OUTTAKING:
                if (Values.init){
                    Values.engaged = 0;
                    Values.spindexerConstants.index=0;
                    Values.init=false;
                }
                //methods.velocityPID(robot.flywheel,methods.calculateVelocity(robot.limelight),"flywheel");
                robot.intake.setPower(0);

                Values.turretConstants.turretPosition += (gamepad1.left_trigger>0)? 20:0;
                Values.turretConstants.turretPosition -= (gamepad1.right_trigger>0)? 20:0;
                Values.turretConstants.turretPosition = Math.min(Values.turretConstants.turretMax, Math.max(Values.turretConstants.turretMin,Values.turretConstants.turretPosition));
                Values.flywheelConstants.flywheelVelocity=1800;

                if (Math.abs(robot.flywheel.getVelocity()-Values.flywheelConstants.flywheelVelocity)<90){
                    robot.led.setPosition(0.444);
                }else{
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
                if (gamepad1.bWasPressed() && Values.engaged==1){
                    Values.engaged = 0;
                    robot.transfer.setPosition(Values.transferBeltStop);
                    robot.transferEngage.setPosition(Values.transferDisengage);
                }

                break;
            case ENDGAME:
                Values.flywheelConstants.flywheelVelocity=0;
                robot.intake.setPower(0);


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
        if (gamepad1.dpadRightWasPressed()) {
            Values.engaged=0;
            Values.spindexerConstants.index += 1;
            Values.spindexerConstants.index %= Values.spindexerConstants.indexer.length;
            methods.resetProfiledPID(Values.spindexerConstants.spindexerPIDF, robot.spindexer);

        }
        if (gamepad1.dpadLeftWasPressed()) {
            Values.engaged=0;
            Values.spindexerConstants.index += 5;
            Values.spindexerConstants.index %= Values.spindexerConstants.indexer.length;
            methods.resetProfiledPID(Values.spindexerConstants.spindexerPIDF, robot.spindexer);

        }

        Values.spindexerConstants.spindexerPosition = Values.spindexerConstants.indexer[Values.spindexerConstants.index];
        methods.velocityPID(robot.flywheel,Values.flywheelConstants.flywheelVelocity, "flywheel");
        methods.positionPID(robot.turret,methods.turretAutoTrack(follower.getPose()),"turret");
        methods.positionPID(robot.spindexer,Values.spindexerConstants.spindexerPosition,"spindexer");
        //turret trig calc
        double x = follower.getPose().getX();
        double y = follower.getPose().getY();
        double heading = follower.getPose().getHeading();


//
//        //Automated PathFollowing
//        if (gamepad1.aWasPressed()) {
//            follower.followPath(pathChain.get());
//            automatedDrive = true;
//        }
//
//        //Stop automated following if the follower is done
//        if (automatedDrive && (gamepad1.bWasPressed() || !follower.isBusy())) {
//            follower.startTeleopDrive();
//            automatedDrive = false;
//        }
//        lt.update();
        telemetry.addData("mode", Values.mode);
        telemetry.addData("index",Values.spindexerConstants.index);
        telemetry.addData("turret",robot.turret.getCurrentPosition());
        telemetry.addData("flywheel",robot.flywheel.getVelocity());
        telemetry.addData("position", follower.getPose());
        telemetry.addData("velocity", follower.getVelocity());

        telemetry.addData("automatedDrive", automatedDrive);
    }

    @Override
    public void stop(){
        Values.spindexerConstants.spindexerPosition=Values.spindexerConstants.spindexerStart;
    }
}