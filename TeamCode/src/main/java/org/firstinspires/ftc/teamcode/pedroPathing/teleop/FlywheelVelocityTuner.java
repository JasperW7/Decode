package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import android.provider.ContactsContract;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.controller.PIDFController;

@Config
@TeleOp(name = "FlywheelVelocityTuner", group = "Tuning")
public class FlywheelVelocityTuner extends LinearOpMode {

    public static double kF = 0.001;
    public static double kP = 0.002;
    public static double kI = 0.0;
    public static double kD = 0.0;
    public static double kV = 0;
    public static double kA = 0;


    public static double targetVelocity = 1500;

    private DcMotorEx flywheel;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private PIDFController controller = new PIDFController(0,0,0,0);

    // PID state
    private double integral = 0;
    private double lastError = 0;
    private ElapsedTime timer = new ElapsedTime();
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
        timer.reset();


        while (opModeIsActive()) {

            double velocity = flywheel.getVelocity();
            controller.setPIDF(kP,kI,kD,kF);
            flywheel.setPower(controller.calculate(velocity,targetVelocity));

            // --- Dashboard telemetry ---
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("TargetVelocity", targetVelocity);
            packet.put("MeasuredVelocity", velocity);
            packet.put("Power", flywheel.getPower());
            dashboard.sendTelemetryPacket(packet);

            // --- Driver station telemetry ---
            telemetry.addData("Target", targetVelocity);
            telemetry.addData("Velocity", velocity);
            telemetry.update();
        }

        flywheel.setPower(0);
    }
}
