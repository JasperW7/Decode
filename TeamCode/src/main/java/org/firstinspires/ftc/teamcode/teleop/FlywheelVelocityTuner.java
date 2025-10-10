package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
@TeleOp(name = "FlywheelVelocityTuner", group = "Tuning")
public class FlywheelVelocityTuner extends LinearOpMode {


    public static double kS = 0.0;
    public static double kV = 0.001;
    public static double kA = 0.0;
    public static double kP = 0.002;
    public static double kI = 0.0;
    public static double kD = 0.0;


    public static double targetVelocity = 2000;

    private DcMotorEx flywheel;
    private FtcDashboard dashboard = FtcDashboard.getInstance();

    // PID state
    private double integral = 0;
    private double lastError = 0;
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        flywheel = hardwareMap.get(DcMotorEx.class, "motor");
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        telemetry.addLine("Flywheel Velocity Tuner Ready");
        telemetry.update();

        waitForStart();
        timer.reset();

        double lastVelocity = 0;

        while (opModeIsActive()) {
            double dt = timer.seconds();
            timer.reset();

            // Read current velocity (ticks per second)
            double velocity = flywheel.getVelocity();

            // Calculate acceleration
            double acceleration = (velocity - lastVelocity) / dt;
            lastVelocity = velocity;

            // --- Feedforward ---
            double ff = kS * Math.signum(targetVelocity)
                    + kV * targetVelocity
                    + kA * acceleration;

            // --- PID ---
            double error = targetVelocity - velocity;
            integral += error * dt;
            double derivative = (error - lastError) / dt;
            lastError = error;

            double feedback = kP * error + kI * integral + kD * derivative;

            // Combine feedforward + feedback
            double power = (ff + feedback) / 12.0; // normalize to motor power (-1 to 1)
            power = Math.max(-1, Math.min(1, power));

            flywheel.setPower(power);

            // --- Dashboard telemetry ---
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("TargetVelocity", targetVelocity);
            packet.put("MeasuredVelocity", velocity);
            packet.put("Error", error);
            packet.put("Feedforward", ff);
            packet.put("Feedback", feedback);
            packet.put("PowerCmd", power);
            dashboard.sendTelemetryPacket(packet);

            // --- Driver station telemetry ---
            telemetry.addData("Target", targetVelocity);
            telemetry.addData("Velocity", velocity);
            telemetry.addData("Power", power);
            telemetry.addData("Error", error);
            telemetry.update();
        }

        flywheel.setPower(0);
    }
}
