package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import static org.firstinspires.ftc.teamcode.pedroPathing.teleop.Values.lerpTable;

import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.LLFieldMap;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.List;
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

        double currentPos = motor.getCurrentPosition();
        double currentTime = System.nanoTime() / 1e9;

        double dt = currentTime - lastTime;
        double dp = currentPos - lastPos;

        double measuredVelocity = dp / dt;

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


    private Methods.DetectedColor lastValidColor = DetectedColor.UNKNOWN;

    public Methods.DetectedColor getDetectedColor(RevColorSensorV3 colorSensor, Telemetry telemetry){

        NormalizedRGBA c = colorSensor.getNormalizedColors();
        double distance = colorSensor.getDistance(DistanceUnit.CM);

        final double BALL_PRESENT_THRESHOLD = 3.0;

        if (distance > BALL_PRESENT_THRESHOLD) {
            return DetectedColor.UNKNOWN;
        }

        float R = c.red / Math.max(c.alpha, 1);
        float G = c.green / Math.max(c.alpha, 1);
        float B = c.blue / Math.max(c.alpha, 1);

        if ((G / R) > 2.0 && G > B) {
            lastValidColor = DetectedColor.GREEN;
        } else if ((B / G) > 1.3 && B > R) {
            lastValidColor = DetectedColor.PURPLE;
        }

        return lastValidColor;
    }



    public enum DetectedColor {
        GREEN,
        PURPLE,
        UNKNOWN
    }

    public double getLimelightDistance(Limelight3A ll){
        return 0; //[some distance that is calculated]
    }
    public double interpolateVelocity(double distance) {
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


    public double turretAutoTrack(Pose botPose) {
        double dy,dx,theta;
        if (Values.team.equals("blue")) {
            dx = botPose.getX() - 12.5;
            dy = 137.3 - botPose.getY();
            theta = 180 - Math.toDegrees(botPose.getHeading())
                    - Math.toDegrees(Math.atan2(dy, dx));
        }else{
            dx = botPose.getX() - 131.5;
            dy = 137.3 - botPose.getY();
            theta = 180 - Math.toDegrees(botPose.getHeading())
                    - Math.toDegrees(Math.atan2(dy, dx));

        }
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

    public double getDistance(Follower follower){
        if (Values.team.equals("blue")){
            return Math.hypot((follower.getPose().getX()-12.5),(follower.getPose().getY()-137.3));
        }else{
            return Math.hypot((follower.getPose().getX()-131.5),(follower.getPose().getY()-137.5));
        }
    }


    private Pose lastGoodPose = null;

    public void relocalize(Limelight3A ll, Follower follower) {
        LLResult result = ll.getLatestResult();
        if (!result.isValid()) return;
        if (result.getFiducialResults().isEmpty()) return;

        Pose3D botpose = result.getBotpose();

        double llX = botpose.getPosition().x;
        double llY = botpose.getPosition().y;
        double llHeading = Math.toRadians(botpose.getOrientation().getYaw());

        Pose llPose = new Pose(llX, llY, llHeading);

        if (lastGoodPose == null) {
            lastGoodPose = llPose;
            follower.setPose(llPose);
            return;
        }

        double dx = Math.abs(llX - lastGoodPose.getX());
        double dy = Math.abs(llY - lastGoodPose.getY());
        if (dx > 0.20 || dy > 0.20) return;

        double alpha = 0.25;
        Pose filteredPose = new Pose(
                lerp(lastGoodPose.getX(), llX, alpha),
                lerp(lastGoodPose.getY(), llY, alpha),
                lerpAngle(lastGoodPose.getHeading(), llHeading, alpha)
        );

        lastGoodPose = filteredPose;
        follower.setPose(filteredPose);
    }

    private double lerp(double a, double b, double t) {
        return a + (b - a) * t;
    }

    private double lerpAngle(double a, double b, double t) {
        double diff = ((b - a + Math.PI) % (2*Math.PI)) - Math.PI;
        return a + diff * t;
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




    //sigma was here

    //TODO: update lerp table for distance
    //TODO: find distance using odo (easy trig)
    //TODO: relocalize with limelight





}


