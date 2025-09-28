package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

@Config
public class Shooter {

    // TODO: enter common field distances in inches (for auto)
    public static double Close = 0;
    public static double Middle = 0;
    public static double Far = 0;

    private static final double GRAVITY = 386.09; // in/s² (imperial gravity)
    private static final double LAUNCH_HEIGHT = 12.0; // inches
    private static final double LAUNCH_ANGLE_RAD = Math.toRadians(54.5);

    // ---- Goal ----
    private static final double GOAL_FRONT_HEIGHT = 39.0; // inches
    private static final double GOAL_BACK_HEIGHT = 54.0;  // inches
    private static final double GOAL_DEPTH = 18.0;        // inches

    // Aim for middle of the goal
    private static final double TARGET_DEPTH = GOAL_DEPTH / 2.0;
    private static final double TARGET_HEIGHT = GOAL_FRONT_HEIGHT
            + (GOAL_BACK_HEIGHT - GOAL_FRONT_HEIGHT) * (TARGET_DEPTH / GOAL_DEPTH);

    // ---- Ball ----
    private static final double BALL_MASS = 0.156 * 0.45359237; // lb to kg

    // ---- Flywheels ----
    private static final double FLYWHEEL_RADIUS = 0.072 / 2.0; //m
    private static final double FLYWHEEL_MASS = 0.056; // kg

    // ---- Motors ----
    private static final double motorTicksPerDegree = 103.8;
    private static final double MOTOR_NO_LOAD_RPM = 1620.0;
    private static final double GEAR_RATIO = 2; // 1:2 gearing up

    // ---- Conversion ----
    private static final double INCH_TO_METER = 0.0254;

    public enum Distances {
        CLOSE(Close),
        MIDDLE(Middle),
        FAR(Far);

        private final double d;

        Distances(double d) {
            this.d = d;
        }

        private double dis() {
            return d;
        }
    }

    public DcMotor leftShooterMotor;
    public DcMotor rightShooterMotor;

    public double lastLeftPosition;
    public double lastRightPosition;

    public double leftRPM;
    public double rightRPM;

    public ElapsedTime RPMTimer;

    public Shooter(HardwareMap HWMap) {
        leftShooterMotor = HWMap.get(DcMotor.class, "leftShooter");
        rightShooterMotor = HWMap.get(DcMotor.class, "rightShooter");

        leftShooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightShooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        RPMTimer.reset();
        lastLeftPosition = 0;
        lastRightPosition = 0;
    }

    public static double calculatePower(double distance) {

        // Target horizontal distance
        double x = distance + TARGET_DEPTH;

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
        double flywheelRPM = (vRim / (2 * Math.PI * FLYWHEEL_RADIUS)) * 60.0;

        // flywheel to motor rpm
        double motorRPM = flywheelRPM / GEAR_RATIO;

        // convert to power fraction
        double power = motorRPM / MOTOR_NO_LOAD_RPM;
        if (power > 1.0) power = 1.0; // clamp

        return power;
    }

    public void updateRPM() {
        leftRPM = (leftShooterMotor.getCurrentPosition() - lastLeftPosition) / RPMTimer.milliseconds() / (motorTicksPerDegree * 360) * GEAR_RATIO * 60000;
        rightRPM = (rightShooterMotor.getCurrentPosition() - lastRightPosition) / RPMTimer.milliseconds() / (motorTicksPerDegree * 360) * GEAR_RATIO * 60000;
        RPMTimer.reset();
        lastLeftPosition = leftShooterMotor.getCurrentPosition();
        lastRightPosition = rightShooterMotor.getCurrentPosition();
    }

    public void setPower(double p) {
        leftShooterMotor.setPower(p);
        rightShooterMotor.setPower(p);
    }

    public class PowerUp implements Action {

        double distance;
        ElapsedTime time;
        boolean init = false;
        double power = 0;

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
                power = calculatePower(distance);
                leftShooterMotor.setPower(power);
                rightShooterMotor.setPower(power);
                time.reset();
                init = true;
            }

            updateRPM();

            return time.milliseconds() > 75 && leftRPM > (power * MOTOR_NO_LOAD_RPM) - (MOTOR_NO_LOAD_RPM * 0.05) && leftRPM < (power * MOTOR_NO_LOAD_RPM) + (MOTOR_NO_LOAD_RPM * 0.1) && rightRPM > power * MOTOR_NO_LOAD_RPM - (MOTOR_NO_LOAD_RPM * 0.05) && rightRPM < power * MOTOR_NO_LOAD_RPM + (MOTOR_NO_LOAD_RPM * 0.1);
        }
    }
    public Action powerUp(double d) {
        return new PowerUp(d);
    }
    public Action powerUp(Distances d) {
        return new PowerUp(d);
    }
}
