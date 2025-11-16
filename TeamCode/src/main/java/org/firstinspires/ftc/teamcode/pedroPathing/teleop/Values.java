package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import com.acmerobotics.dashboard.config.Config;
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
        public static double sP = 0.01;
        public static double sI = 0.13;
        public static double sD = 0.0003;
        public static double sK = 0;
        public static double sV =3000;
        public static double sA = 1000;
        public static double spindexerPosition=0;
        public static final double spindexerStart = 0, spindexerGreen=100, spindexerPurple1=230, spindexerPurple2=357, spindexerGreenTransfer = 25,spindexerPurpleTransfer1 = 164, spindexerPurpleTransfer3 = 290;
        public static final double[]indexer = new double[] {spindexerGreen,spindexerPurple1,spindexerPurple2,spindexerGreenTransfer,spindexerPurpleTransfer1,spindexerPurpleTransfer3};
        public static int index = 0;
    }


    public static final class turretConstants {
        public static final ProfiledPIDController turretPIDF = new ProfiledPIDController(0, 0, 0, new TrapezoidProfile.Constraints(0,0));
        public static double tP = 0.006;
        public static double tI = 0.12;
        public static double tD = 0.0003;
        public static double tK = 0;
        public static double tV = 8000;
        public static double tA = 6000;
        public static double turretPosition=0;

        public static double turretMin = -1100, turretStart = 0,turretMax = 1100;
    }


    public enum Mode {
        INTAKING,
        OUTTAKING,
        ENDGAME
    }




    public static Mode mode = Mode.OUTTAKING;

    // Example mechanism positions


    public static final TreeMap<Double, Integer> lerpTable = new TreeMap<>(
            // distance, target velocity
            Map.ofEntries(
                    Map.entry(2.13, 2100),
                    Map.entry(2.54, 2000),
                    Map.entry(3.81, 2000),
                    Map.entry(4.5, 2000),
                    Map.entry(4.7, 2000)
            )
    );
    public static String motif;
    public static String team;
    public static double transferBeltStart = 1, transferBeltStop = 0.5, transferBeltMid = 0.8;
    public static double transferDisengage=0.35, transferEngage=0.6, transferKick=0.88;


    public static double engageTarget = 0.0;   // where we WANT it to go
    public static double engageStep = 0.002;   // how much to move per loop
    public static double engagePos = 0.0;      // TRACKED servo position

    public static boolean intaking = false;
    public static boolean init = true;
    public static int engaged = 0;

}
