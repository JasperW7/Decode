package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;

import java.util.Map;
import java.util.TreeMap;
@Config
public final class Values {


    public static final class flywheelConstants {
        public static final PIDFController flywheelPIDF = new PIDFController(0, 0, 0, 0);
        public static double fP = 0.01;
        public static double fI = 0;
        public static double fD = 0.00005;
        public static double fK = 0.000445;
        public static double fV  = 0;
        public static double fA = 0;

        public static double flywheelVelocity=1800;
    }

    public static final class intakeConstants {
        public static final PIDFController intakePIDF = new PIDFController(0, 0, 0, 0);
        public static double iP = 0.0005;
        public static double iI = 0;
        public static double iD = 0.000015;
        public static double iK = 0.0004;
        public static double iV = 0;
        public static double iA = 0;

        public static double intakeVelocity = 500;
    }


    public static final class spindexerConstants {
        public static final ProfiledPIDController spindexerPIDF = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0,0));
        public static double sP = -0.00025;
        public static double sI = -0.1;
        public static double sD = -0.00003;
        public static double sK = 0;
        public static double sV =200000;
        public static double sA = 200000;
        public static double spindexerPosition=0;
        public static final double spindexerStart = 0, spindexerGreen=2100, spindexerPurple1=4950, spindexerPurple2=7800, spindexerGreenTransfer = 6200,spindexerPurpleTransfer1 = 800, spindexerPurpleTransfer3 = 3600; //forward -> 800, 3600, 6200; transfer -> 2100, 4950, 7600
        public static final double[]indexer = new double[] {spindexerGreen,spindexerPurple1,spindexerPurple2,spindexerGreenTransfer,spindexerPurpleTransfer1,spindexerPurpleTransfer3};
        public static int index = 0;
    }


    public static final class turretConstants {
        public static final ProfiledPIDController turretPIDF = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0,0));
        public static double tP = 0.006;
        public static double tI = 0.12;
        public static double tD = 0.0003;
        public static double tK = 0;
        public static double tV = 0;
        public static double tA = 0;
        public static double turretPosition=0;

        public static double turretMin = -1100, turretStart = 0,turretMax = 1100;
    }


    public enum Mode {
        INTAKING,
        OUTTAKING,
        ENDGAME
    }




    public static Mode mode = Mode.INTAKING;

    // Example mechanism positions


    public static final TreeMap<Double, Integer> lerpTable = new TreeMap<>(
            // distance, target velocity
            Map.ofEntries(
                    Map.entry(38.386, 1500),
                    Map.entry(64.55,1670),
                    Map.entry(74.0316, 1770),
                    Map.entry(84.1,1800)
            )
    );
    public static String motif="";
    public static String team="blue";
    public static double transferBeltStart = 1, transferBeltStop = 0.5, transferBeltMid = 0.8;
    public static double transferDisengage=0.35, transferEngage=0.6, transferKick=0.88;


    public static int greenCount=0;
    public static int purpleCount =0;
    public static boolean purpleBallProcessed = false;
    public static boolean greenBallProcessed = false;
    public static boolean waitingOnSpindex = false;

    public static boolean reversingIntake = false;
    public static Methods.DetectedColor lastDetectedColor = Methods.DetectedColor.UNKNOWN;
    public static Methods.DetectedColor lastProcessedColor = Methods.DetectedColor.UNKNOWN;
    public static Methods.DetectedColor reversingColor = Methods.DetectedColor.UNKNOWN;


    public static boolean waitingForBallToLeave = false;
    public static int lastColorFrames=0;


    public static boolean drivers=false;

    public static boolean init = true;
    public static int engaged = 0;
    public static boolean endgame = false;

}
