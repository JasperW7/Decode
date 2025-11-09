package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.arcrobotics.ftclib.controller.PIDFController;

@Config
@TeleOp(name = "Position Tuner", group = "Tuning")
public class PositionTuner extends LinearOpMode {

    public static double kF = 0;
    public static double kP = 0.008;
    public static double kI = 0.0;
    public static double kD = 0.0001;
    public static double maxVel = 0;
    public static double maxAccel = 0;
    public static double multi = 0.5;


    public static double targetPosition = 0;

    private DcMotorEx motor;

    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private ProfiledPIDController controller = new ProfiledPIDController(0,0,0,new TrapezoidProfile.Constraints(0,0));

    // PID state
    private double integral = 0;
    private double lastError = 0;
    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode() {
        motor = hardwareMap.get(DcMotorEx.class, "spindexer");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addLine("Position Tuner Ready");
        telemetry.update();

        waitForStart();
        timer.reset();


        while (opModeIsActive()) {

            double currPos = motor.getCurrentPosition();
            controller.setPID(kP,kI,kD);
            controller.setConstraints(new TrapezoidProfile.Constraints(maxVel,maxAccel));
            motor.setPower(controller.calculate(currPos,targetPosition));
            // --- Dashboard telemetry ---
            TelemetryPacket packet = new TelemetryPacket();
            packet.put("Target Position", targetPosition);
            packet.put("Current Position", currPos);
            dashboard.sendTelemetryPacket(packet);

            // --- Driver station telemetry ---

            telemetry.addData("Target", targetPosition);
            telemetry.addData("Position", currPos);
            telemetry.update();
        }

        motor.setPower(0);
    }
}
