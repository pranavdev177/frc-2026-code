package frc.robot;

public class Constants {
    public final class Drive {
        public static final double MAX_ANGULAR_RATE_RPS = 0.75;
        public static final double DEADBAND_PERCENT = 0.1;
        public static final double MIN_SPEED = 2.5;
        public static final double MAX_SPEED = 4.5;
        public static final double SPEED_INCREMENT = 0.5;
    }

    public final class Launcher {
        // Speed
        public static final double DEFAULT_SPEED = 0.8;
        public static final double MIN_SPEED = 0.6;
        public static final double MAX_SPEED = 1.0;
        public static final double SPEED_INCREMENT = 0.05;
        public static final double LONG_SHOOT_AUTO_SPEED = 0.8;
        public static final double SHORT_SHOOT_AUTO_SPEED = 0.6;

        public static final double SPINUP_TIMEOUT_SECONDS = 2;

        // PID
        public static final double kP = 0.1;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kV = 0.12;
        public static final double kS = 0.2;        
    }

    public final class Intake {
        // Current Limits
        public static final double STATOR_CURRENT_LIMIT = 30;

        //Up down cycle for compacting
        public static final double COMPACT_TIME = 0.5;

        // Speed
        public static final double INTAKE_SPEED = 0.7;

        //Intake angle speed
        public static final double INTAKE_ANGLE_SPEED = 0.3;

        //Max and min intake angle
        public static final double INTAKE_ANGLE_MAX = 90;
        public static final double INTAKE_ANGLE_MIN = 0;

        // Spin PID
        public static final double Spin_kP = 1.0;
        public static final double Spin_kI = 0.0;
        public static final double Spin_kD = 0.0;
        public static final double Spin_kV = 0.5;
        public static final double Spin_kS = 0.2;  
        
        //Angle PID
        public static final double Angle_kP = 50;
        public static final double Angle_kI = 0.0;
        public static final double Angle_kD = 0.1;
        public static final double Angle_kV = 0.5;
        public static final double Angle_kS = 0.25;  
        public static final double Angle_kG = 0.3;
    }

    public final class Indexer {
        public static final double INDEXER_SPEED = 0.8;
    }

    public final class Match {
        public static final int MATCH_DURATION_SECONDS = 160;
        public static final int[] ROUND_END_TIMES = {140, 105, 80, 55, 30, 0};
    }

    public final class Testing {
        public static final boolean TESTING = false;
        public static final boolean ENABLE_LOGS = true;
    }
}
