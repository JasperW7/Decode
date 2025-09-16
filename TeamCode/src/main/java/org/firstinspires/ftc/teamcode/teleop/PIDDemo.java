package org.firstinspires.ftc.teamcode.teleop;


import com.arcrobotics.ftclib.controller.PIDFController;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.graph.GraphManager;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.graph.PanelsGraph;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@Configurable
@TeleOp
public class PIDDemo extends LinearOpMode {
    private DcMotorEx motor;


    PIDFController pidf = new PIDFController(0,0,0,0);

    public static double P=0.0,I=0,D=0.0,F=0;
    public static double targetVel = 0;



    @IgnoreConfigurable
    static TelemetryManager telemetryM;
    static GraphManager graphM;


    @Override
    public void runOpMode(){

        motor = hardwareMap.get(DcMotorEx.class,"motor");
        motor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        graphM = PanelsGraph.INSTANCE.getManager();
        waitForStart();
        while(opModeIsActive()){
            targetVel = Math.min(Math.max(0,targetVel),1000);
            motor.setVelocityPIDFCoefficients(P,I,D,F);
            motor.setVelocity(targetVel);

            telemetryM.debug("target" + targetVel);
            telemetryM.debug("position" + motor.getCurrentPosition());
            telemetryM.debug("power" + motor.getPower());
            graphM.addData("target",targetVel);
            graphM.addData("position",motor.getCurrentPosition());
            graphM.addData("power",motor.getPower());
            graphM.update();
            telemetryM.update(telemetry);


        }
    }


    public double PIDF(double target){

        motor.setVelocityPIDFCoefficients(P,I,D,F);
        double curr = motor.getVelocity();
        double output = pidf.calculate(curr,target);
        return output;

    }
}
