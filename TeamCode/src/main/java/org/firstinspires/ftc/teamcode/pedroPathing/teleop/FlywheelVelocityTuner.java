package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "FlywheelVelocityTuner", group = "Tuning")
public class FlywheelVelocityTuner extends LinearOpMode {

    // Dashboard-tunable target velocity
    public static double targetVelocity = 1500;

    // These update the Values.flywheelConstants dynamically
    public static double fP = 0.002;
    public static double fI = 0.0;
    public static double fD = 0.0;
    public static double fK = 0.001;
    public static double fV = 0;
    public static double fA = 0;

    private DcMotorEx flywheel;
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private Methods methods = new Methods();

    @Override
    public void runOpMode() {

        flywheel = hardwareMap.get(DcMotorEx.class, "intake");
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("Flywheel Velocity Tuner Ready");
        telemetry.update();

        waitForStart();

        // Reset PID loop state in your velocityPID() code
        methods.resetVelocityPID();

        while (opModeIsActive()) {

            // Update the actual constants your Methods.velocityPID() uses
            Values.intakeConstants.iP = fP;
            Values.intakeConstants.iI = fI;
            Values.intakeConstants.iD = fD;
            Values.intakeConstants.iK = fK;
            Values.intakeConstants.iV = fV;
            Values.intakeConstants.iA = fA;

            Values.flywheelConstants.flywheelPIDF.setPIDF(fP, fI, fD, fK);

            // ---- RUN YOUR ACTUAL LOGIC ----
            methods.velocityPID(flywheel, targetVelocity, "intake");

            double measuredVelocity = flywheel.getVelocity();

            // === Dashboard Telemetry ===
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("TargetVelocity", targetVelocity);
            packet.put("MeasuredVelocity", measuredVelocity);
            packet.put("Power", flywheel.getPower());
            dashboard.sendTelemetryPacket(packet);

            // === Driver Station Telemetry ===
            telemetry.addData("Target", targetVelocity);
            telemetry.addData("Velocity", measuredVelocity);
            telemetry.update();
        }

        flywheel.setPower(0);
    }
}
