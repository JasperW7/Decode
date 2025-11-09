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

import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import java.util.function.Supplier;

@Configurable
@TeleOp
public class Teleop extends OpMode {
    private Hardware robot;

    private Follower follower;
    public static Pose startingPose;
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
                .addPath(new Path(new BezierLine(follower::getPose, new Pose(45, 98))))
                .setHeadingInterpolation(HeadingInterpolator.linearFromPoint(follower::getHeading, Math.toRadians(45), 0.8))
                .build();
    }

    @Override
    public void init_loop(){
        /** This method is called continuously after Init while waiting for "play". **/
        methods.positionPID(robot.turret,Values.turretConstants.turretStart,"turret");
        methods.positionPID(robot.spindexer,Values.spindexerConstants.spindexerStart,"spindexer");
        methods.velocityPID(robot.flywheel,Values.flywheelConstants.flywheelVelocity,"flywheel");
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
                        -gamepad2.left_stick_y*0.5,
                        -gamepad2.left_stick_x*0.5,
                        -gamepad2.right_stick_x*0.5,
                        true // Robot Centric
                );
            }else {
                follower.setTeleOpDrive(
                        -gamepad2.left_stick_y,
                        -gamepad2.left_stick_x,
                        -gamepad2.right_stick_x,
                        true // Robot Centric
                );
            }
        }

        if (gamepad1.leftBumperWasPressed()){
            if (Values.mode == Values.Mode.OUTTAKING){
                Values.mode = Values.Mode.INTAKING;
            }
        }
        if (gamepad1.rightBumperWasPressed()){
            if (Values.mode == Values.Mode.INTAKING){
                Values.mode = Values.Mode.OUTTAKING;
            }else if (Values.mode == Values.Mode.OUTTAKING){
                Values.mode = Values.Mode.ENDGAME;
            }else if (Values.mode == Values.Mode.ENDGAME){
                Values.mode = Values.Mode.OUTTAKING;
            }
        }


        switch (Values.mode){
            case INTAKING:

                Values.turretConstants.turretPosition = Values.turretConstants.turretStart;
                robot.intake.setPower(1);


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
                //methods.velocityPID(robot.flywheel,methods.calculateVelocity(robot.limelight),"flywheel");
                robot.intake.setPower(0);
                Values.turretConstants.turretPosition += (gamepad1.left_trigger>0)? 20:0;
                Values.turretConstants.turretPosition -= (gamepad1.right_trigger>0)? 20:0;
                Values.turretConstants.turretPosition = Math.min(Values.turretConstants.turretMax, Math.max(Values.turretConstants.turretMin,Values.turretConstants.turretPosition));

                if (Math.abs(robot.flywheel.getVelocity()-Values.flywheelConstants.flywheelVelocity)<50){
                    robot.led.setPosition(0.444);
                }else{
                    robot.led.setPosition(0);
                }

                break;
            case ENDGAME:
                robot.intake.setPower(0);


        }


        if (gamepad1.aWasPressed()){
            switch (Values.engaged){
                case 0:
                    Values.engaged = 1;
                    robot.transfer.setPosition(Values.transferBeltStart);
                    robot.transferEngage.setPosition(Values.transferEngage);
                    break;
                case 1:
                    Values.mode = Values.Mode.OUTTAKING;
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

        if (gamepad1.dpad_left || gamepad1.dpad_right){
            Values.spindexerConstants.override = true;
        }
        if (Values.spindexerConstants.override) {
            if (gamepad1.b || gamepad1.x) {
                Values.spindexerConstants.override = false;
            }
            Values.spindexerConstants.spindexerPosition += (gamepad1.dpad_right) ? 20 : 0;
            Values.spindexerConstants.spindexerPosition -= (gamepad1.dpad_left) ? 20 : 0;
        }else{
            Values.spindexerConstants.index += (gamepad1.bWasPressed()) ? (((Values.spindexerConstants.index + 1) % 6))%6 : 0;
            Values.spindexerConstants.index += (gamepad1.xWasPressed()) ? (((Values.spindexerConstants.index + 5) % 6))%6 : 0;
            Values.spindexerConstants.index %= 6;
            Values.spindexerConstants.spindexerPosition = Values.spindexerConstants.indexer[Values.spindexerConstants.index];

        }
        methods.velocityPID(robot.flywheel,Values.flywheelConstants.flywheelVelocity, "flywheel");
        methods.positionPID(robot.turret,Values.turretConstants.turretPosition,"turret");
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
        telemetry.addData("override",Values.spindexerConstants.override);

        telemetry.addData("spindexer",robot.spindexer.getCurrentPosition());
        telemetry.addData("turret",robot.turret.getCurrentPosition());
        telemetry.addData("intake", robot.intake.getVelocity());
        telemetry.addData("flywheel",robot.flywheel.getVelocity());
        telemetry.addData("position", follower.getPose());
        telemetry.addData("velocity", follower.getVelocity());
        telemetry.addData("automatedDrive", automatedDrive);
    }
}