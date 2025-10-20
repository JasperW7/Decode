package org.firstinspires.ftc.teamcode.pedroPathing.teleop;
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
        switch (Values.mode){
            case INTAKING:
                if (gamepad1.leftBumperWasPressed()){
                    Values.intaking = !Values.intaking;
                }


                Methods.DetectedColor color = methods.getDetectedColor(robot.colorSensor, telemetry);

                if (color != Methods.DetectedColor.UNKNOWN && color != robot.lastColor) {
                    robot.lastColor = color;

                    if (Values.mode == Values.Mode.INTAKING && color != Methods.DetectedColor.GREEN) {
                        robot.led.setPosition(0.722);
                    } else if (Values.mode == Values.Mode.INTAKING) {
                        robot.led.setPosition(0.444);
                    } else {
                        robot.led.setPosition(0);
                    }
                }



                break;
            case OUTTAKING:
                //methods.velocityPID(robot.flywheel,methods.calculateVelocity(robot.limelight),"flywheel");
                methods.velocityPID(robot.flywheel,2000,"flywheel");
                break;

        }

        if (Values.intaking){
            methods.velocityPID(robot.intake,Values.intakeConstants.intakeVelocity, "intake");
        }else{
            robot.intake.setPower(0);
        }

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


        telemetryM.debug("position", follower.getPose());
        telemetryM.debug("velocity", follower.getVelocity());
        telemetryM.debug("automatedDrive", automatedDrive);
    }
}