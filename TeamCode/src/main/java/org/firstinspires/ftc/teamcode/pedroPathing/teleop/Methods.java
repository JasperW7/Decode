package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import static org.firstinspires.ftc.teamcode.pedroPathing.teleop.Values.lerpTable;

import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Map;

public class Methods {

    public void velocityPID(DcMotorEx motor, double targetVelocity, String mechanismType) {
        PIDFController controller;
        double kP, kI, kD, kF;
        switch(mechanismType){
            case "flywheel":
                kP = Values.flywheelConstants.fP;
                kI = Values.flywheelConstants.fI;
                kD = Values.flywheelConstants.fD;
                kF = Values.flywheelConstants.fK;
                controller = Values.flywheelConstants.flywheelPIDF;
                break;
            case "intake":
                kP = Values.intakeConstants.iP;
                kI = Values.intakeConstants.iI;
                kD = Values.intakeConstants.iD;
                kF = Values.intakeConstants.iK;
                controller = Values.intakeConstants.intakePIDF;
                break;
            default:
                throw new IllegalArgumentException("Error: " + mechanismType);
        }
        controller.setPIDF(kP,kI,kD,kF);
        double currentVelocity = motor.getVelocity();
        double power = controller.calculate(currentVelocity, targetVelocity);
        motor.setPower(power);

    }

    public void positionPID(DcMotorEx motor, double targetPosition, String mechanismType) {
        PIDFController controller;
        double kP, kI, kD, kF;

        switch (mechanismType) {
            case "turret":
                kP = Values.turretConstants.tP;
                kI = Values.turretConstants.tI;
                kD = Values.turretConstants.tD;
                kF = Values.turretConstants.tK;
                controller = Values.turretConstants.turretPIDF;
                break;

            case "spindexer":
                kP = Values.spindexerConstants.sP;
                kI = Values.spindexerConstants.sI;
                kD = Values.spindexerConstants.sD;
                kF = Values.spindexerConstants.sK;
                controller = Values.spindexerConstants.spindexerPIDF;
                break;
            default:
                throw new IllegalArgumentException("Error: " + mechanismType);
        }
        controller.setPIDF(kP, kI, kD, kF);
        double position = motor.getCurrentPosition();
        double power = controller.calculate(position, targetPosition);
        motor.setPower(power);
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




}


