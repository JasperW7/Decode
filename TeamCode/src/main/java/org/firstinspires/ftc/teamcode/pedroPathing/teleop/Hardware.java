package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import com.arcrobotics.ftclib.hardware.motors.MotorEx;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.arcrobotics.ftclib.hardware.ServoEx;

import org.firstinspires.ftc.robotcontroller.external.samples.SensorGoBildaPinpoint;

public class Hardware {

    public DcMotorEx flywheel;
    public DcMotorEx intake;
    public DcMotorEx spindexer;
    public DcMotorEx turret;

    public ServoEx transfer;
    public ServoEx transferEngage;
    public ServoEx shooterUnder;
    public ServoEx ejection;
    public ServoEx kickstand;
    public ServoEx led;

    public RevColorSensorV3 colorSensor;
    public Limelight3A limelight;

    private HardwareMap hwMap;

    public  Methods.DetectedColor lastColor = Methods.DetectedColor.UNKNOWN;

    public Hardware(HardwareMap hardwareMap) {
        this.hwMap = hardwareMap;
    }

    public void init() {
        flywheel = hwMap.get(DcMotorEx.class,"flywheel");
        intake = hwMap.get(DcMotorEx.class,"intake");
        spindexer = hwMap.get(DcMotorEx.class,"spindexer");
        turret = hwMap.get(DcMotorEx.class,"turret");

        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheel.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheel.setPower(0);

        intake.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intake.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake.setPower(0);

        spindexer.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        spindexer.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        spindexer.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        spindexer.setPower(0);

        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        turret.setPower(0);

        transfer = hwMap.get(ServoEx.class,"transfer");
        transferEngage = hwMap.get(ServoEx.class,"engage");
        shooterUnder = hwMap.get(ServoEx.class,"shooter");
        ejection = hwMap.get(ServoEx.class,"ejection");
        kickstand = hwMap.get(ServoEx.class,"kickstand");
        led = hwMap.get(ServoEx.class,"led");

        limelight = hwMap.get(Limelight3A.class,"limelight");
        colorSensor = hwMap.get(RevColorSensorV3.class, "colorSensor");


    }
}
