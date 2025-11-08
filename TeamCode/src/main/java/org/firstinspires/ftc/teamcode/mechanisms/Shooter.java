package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Shooter {

    public static double kP = 0.001;
    public static double kI = 0;
    public static double kD = 0;
    public static double kF = 0.00027777;

    public static double shootingConstant = 300;

    public static double thresholdTol = 65;

    // TODO: enter common field distances in inches (for auto)
    //common field distances
    public static double Close = 0;
    public static double Middle = 63;
    public static double Far = 0;

    private static final double GRAVITY = 386.09; // in/s² (imperial gravity)
    private static final double LAUNCH_HEIGHT = 13.5; // inches
    private static final double LAUNCH_ANGLE_RAD = Math.toRadians(50);

    // ---- Goal ----
    private static final double GOAL_FRONT_HEIGHT = 39.0; // inches
    private static final double GOAL_BACK_HEIGHT = 54.0;  // inches
    private static final double GOAL_DEPTH = 18.0;        // inches

    // Aim for middle of the goal
    private static final double TARGET_DEPTH = GOAL_DEPTH / 2.0;
    private static final double TARGET_HEIGHT = GOAL_FRONT_HEIGHT + (GOAL_BACK_HEIGHT - GOAL_FRONT_HEIGHT) * (TARGET_DEPTH / GOAL_DEPTH);

    // ---- Ball ----
    private static final double BALL_MASS = 0.156 * 0.45359237; // lb to kg

    // ---- Flywheels ----
    private static final double FLYWHEEL_RADIUS = 0.072 / 2.0; //m
    private static final double FLYWHEEL_MASS = 0.056; // kg

    // ---- Motors ----
    public static final double motorTicksPerRevolution = 103.8;
    public static final double MOTOR_NO_LOAD_RPM = 1300; //1620.0
    public static final double GEAR_RATIO = 2.5; // 1:2.5 gearing up

    // ---- Conversion ----
    private static final double INCH_TO_METER = 0.0254;

    public enum Distances {
        //set each state to the correct field distances
        CLOSE(Close),
        MIDDLE(Middle),
        FAR(Far);

        private final double d;

        //constructor for each state
        Distances(double d) {
            this.d = d;
        }

        //method to get the distance from the enum
        private double dis() {
            return d;
        }
    }

    public DcMotor shooterMotor;

    public double lastPosition;
    public double RPM;


    public ElapsedTime RPMTimer;

    public PIDFController PIDF;

    public double targetRPM = 0;

    public Shooter(HardwareMap HWMap) {
        shooterMotor = HWMap.get(DcMotor.class, "rightShooter");

        shooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        shooterMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        RPMTimer = new ElapsedTime();
        RPMTimer.reset();

        PIDF = new PIDFController(kP, kI, kD, kF);
    }

    public static double calculateRPM(double distance) {
        if (distance < 81) {
            // Target horizontal distance
            double depthSigmoid = GOAL_DEPTH / (1.0 + Math.exp(0.3 * (distance - 46)));
            double x = distance + depthSigmoid;

            // Target vertical distance
            double y = TARGET_HEIGHT - LAUNCH_HEIGHT;

            // required launch speed
            // y = x*tanθ - g*x²/(2*v²*cos²θ)
            double theta = LAUNCH_ANGLE_RAD;
            double tanTheta = Math.tan(theta);
            double cosTheta = Math.cos(theta);

            double numerator = GRAVITY * x * x;
            double denominator = 2.0 * cosTheta * cosTheta * (x * tanTheta - y);
            double vRequired_in_per_s = Math.sqrt(numerator / denominator);

            // convert to m/s
            double vRequired_m_per_s = vRequired_in_per_s * INCH_TO_METER;

            // ball exit velocity to flywheel rim speed
            double vRim = vRequired_m_per_s * (BALL_MASS + FLYWHEEL_MASS) / FLYWHEEL_MASS;

            // rim speed to flywheel rpm
            double flywheelRPM =  (vRim / (2 * Math.PI * FLYWHEEL_RADIUS)) * 60.0;

            return Math.min(flywheelRPM + shootingConstant, 3650);
        } else {
            return -46.20253 * distance + 7392.40506;
        }

    }


    public boolean RPMInThreshold() {
        return Math.abs(targetRPM - RPM) < thresholdTol;
    }

    public void updateRPM() {
        double currentTime = RPMTimer.milliseconds();

        double currentRightRPM = ((shooterMotor.getCurrentPosition() - lastPosition) / motorTicksPerRevolution) * GEAR_RATIO * (60000  / currentTime);

        RPM = currentRightRPM;

        RPMTimer.reset();

        lastPosition = shooterMotor.getCurrentPosition();

        PIDF.setPIDF(kP, kI, kD, kF);
    }

    public void updatePID() {
        shooterMotor.setPower(PIDF.calculate(RPM, targetRPM));
    }

    public class PowerUp implements Action {

        double distance;
        ElapsedTime time;
        boolean init = false;

        public PowerUp(double distance) {
            this.distance = distance;
            time = new ElapsedTime();
        }

        public PowerUp(Distances distance) {
            this.distance = distance.dis();
            time = new ElapsedTime();
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!init) {
                //targetRPM = calculateRPM(distance);
                //targetRPM = 3500;
                time.reset();
                init = true;
            }

            //updateRPM();
            //updatePID();

            return false;
            //return (time.milliseconds() > 75 && RPMInThreshold());

        }
    }

    public Action powerUp(double d) {
        return new PowerUp(d);
    }
    public Action powerUp(Distances d) {
        return new PowerUp(d);
    }

    public class Stop implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            shooterMotor.setPower(0);
            return false;
        }

    }
    public Action stop() {
        return new Stop();
    }

    public class Run implements Action {
        private boolean init = false;
        private double target;

        public Run(double target) {
            this.target = target;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!init) {
                RPMTimer.reset();
                init = true;
            }

            targetRPM = target;

            updateRPM();
            updatePID();

            return true;
        }
    }

    public Action run(double target) {
        return new Run(target);
    }

}