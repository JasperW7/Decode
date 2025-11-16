package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import static org.firstinspires.ftc.teamcode.pedroPathing.teleop.Values.lerpTable;

import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;

public class Methods {
    private double lastPos = 0;
    private double lastTime = 0;
    private boolean firstLoop = true;
    public void velocityPID(DcMotorEx motor, double targetVelocity, String mechanismType) {
        PIDFController controller;
        double kP, kI, kD, kF, kV, kA;

        switch (mechanismType) {
            case "flywheel":
                kP = Values.flywheelConstants.fP;
                kI = Values.flywheelConstants.fI;
                kD = Values.flywheelConstants.fD;
                kF = Values.flywheelConstants.fK;
                kV = Values.flywheelConstants.fV;
                kA = Values.flywheelConstants.fA;
                controller = Values.flywheelConstants.flywheelPIDF;
                break;

            case "intake":
                kP = Values.intakeConstants.iP;
                kI = Values.intakeConstants.iI;
                kD = Values.intakeConstants.iD;
                kF = Values.intakeConstants.iK;
                kV = Values.intakeConstants.iV;
                kA = Values.intakeConstants.iA;
                controller = Values.intakeConstants.intakePIDF;
                break;

            default:
                throw new IllegalArgumentException("Error: " + mechanismType);
        }

        if (firstLoop) {
            lastPos = motor.getCurrentPosition();
            lastTime = System.nanoTime() / 1e9;
            firstLoop = false;
            return;
        }

        // compute delta-position velocity
        double currentPos = motor.getCurrentPosition();
        double currentTime = System.nanoTime() / 1e9;

        double dt = currentTime - lastTime;
        double dp = currentPos - lastPos;

        double measuredVelocity = dp / dt;   // ticks per second

        lastPos = currentPos;
        lastTime = currentTime;

        if (targetVelocity == 0) {
            motor.setPower(0);
            return;
        }

        controller.setPIDF(kP, kI, kD, kF);
        double power = controller.calculate(measuredVelocity, targetVelocity);
        motor.setPower(power);
    }
    public void resetVelocityPID() {
        firstLoop = true;
    }




    public void positionPID(DcMotorEx motor, double targetPosition, String mechanismType) {
        ProfiledPIDController controller;
        double kP, kI, kD, kF, kV, kA;

        switch (mechanismType) {
            case "turret":
                kP = Values.turretConstants.tP;
                kI = Values.turretConstants.tI;
                kD = Values.turretConstants.tD;
                kF = Values.turretConstants.tK;
                kV = Values.turretConstants.tV;
                kA = Values.turretConstants.tA;
                controller = Values.turretConstants.turretPIDF;
                break;

            case "spindexer":
                kP = Values.spindexerConstants.sP;
                kI = Values.spindexerConstants.sI;
                kD = Values.spindexerConstants.sD;
                kF = Values.spindexerConstants.sK;
                kV = Values.spindexerConstants.sV;
                kA = Values.spindexerConstants.sA;
                controller = Values.spindexerConstants.spindexerPIDF;
                break;
            default:
                throw new IllegalArgumentException("Error: " + mechanismType);
        }
        controller.setPID(kP, kI, kD);
        controller.setConstraints(new TrapezoidProfile.Constraints(kV,kA));
        double position = motor.getCurrentPosition();
        double power = controller.calculate(position, targetPosition);
        motor.setPower(power);
    }
    public void resetProfiledPID(ProfiledPIDController controller, DcMotorEx motor) {
        controller.reset(new TrapezoidProfile.State(motor.getCurrentPosition(), 0));
    }


    public DetectedColor getDetectedColor(RevColorSensorV3 colorSensor, Telemetry telemetry){

        NormalizedRGBA colors= colorSensor.getNormalizedColors();

        float normRed, normGreen, normBlue;
        normRed = colors.red/colors.alpha;
        normGreen = colors.green/colors.alpha;
        normBlue = colors.blue/colors.alpha;


        /**
         *red, green, blue
         * GREEN =
         * PURPLE =
         */
        telemetry.addData("red",normRed);
        telemetry.addData("green",normGreen);
        telemetry.addData("blue",normBlue);

        if (normRed>0 && normGreen>0 && normBlue>0){ //swap with purple range
            return DetectedColor.PURPLE; //purple
        }else if (normRed>0.5 && normGreen>1 && normBlue>1){ //swap with green range
            return DetectedColor.GREEN; //green
        }else{
            return DetectedColor.UNKNOWN;
        }
    }

    public enum DetectedColor {
        GREEN,
        PURPLE,
        UNKNOWN
    }

    public double getLimelightDistance(Limelight3A ll){
        return 0; //[some distance that is calculated]
    }
    private double interpolateVelocity(double distance) {
        if (lerpTable.containsKey(distance)) return lerpTable.get(distance);

        Map.Entry<Double, Integer> lower = lerpTable.floorEntry(distance);
        Map.Entry<Double, Integer> higher = lerpTable.ceilingEntry(distance);

        if (lower == null) {
            assert higher != null;
            return higher.getValue();
        }
        if (higher == null) return lower.getValue();

        double d1 = lower.getKey(), d2 = higher.getKey();
        double r1 = lower.getValue(), r2 = higher.getValue();

        double ratio = (distance - d1) / (d2 - d1);
        return r1 + (r2 - r1) * ratio;
    }

    public double calculateVelocity(Limelight3A ll){
        double distance = getLimelightDistance(ll);
        return interpolateVelocity(distance);

    }

    public double turretAutoTrack(Pose botPose) {
        double dy,dx;
        if (Values.team.equals("blue")) {
            dx = botPose.getX() - 12.5;
            dy = 137.3 - botPose.getY();
        }else{
            dx = 131.5-botPose.getX();
            dy= 137.3-botPose.getY();
        }
        double theta = 180 - Math.toDegrees(botPose.getHeading())
                - Math.toDegrees(Math.atan2(dy, dx));
        double ticks = 55 * theta / 9;
        double wrapped = ((ticks + 1100) % 2200);
        if (wrapped < 0) wrapped += 2200;
        wrapped -= 1100;
        return wrapped;
    }

    public void manualRelocalize(Follower follower){
        if (Values.team.equals("red")) {
            follower.setPose(new Pose(9.5, 9.5, Math.toRadians(90)));
        }else{
            follower.setPose(new Pose(134.5,9.5,Math.toRadians(90)));
        }
    }


//    public double[] relocalize(Limelight3A ll){
//        return [x,y,heading];
//    }

    //sigma was here

    //TODO: update lerp table for distance
    //TODO: find distance using limelight
    //TODO: relocalize with limelight
    //TODO: turret tracking using trig
    //TODO: find color ranges





}


