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

    // TODO: 0.5 * flywheel mass (kg) * (flywheel radius)^2 (m)
    final static double flywheelInertia = 0.5 * 1 * 2 * 2;
    // TODO: set max motor speed at 12V
    final static double motorMaxRPM = 6000;
    // TODO: how much speed is conserved
    final static double gearEfficiency = 1;
    // TODO: gear ratio is motor speed / flywheel speed
    final static double gearRatio = 3;
    // TODO: set to motor torque (at 12V when stalled)
    final static double motorStallTorque = 0.35;
    static double RECOVERY_CONSTANT = (flywheelInertia * motorMaxRPM * 2 * Math.PI / 60.0) / (gearEfficiency * gearRatio * gearRatio * motorStallTorque);

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

    public Shooter(HardwareMap HWMap) {
        leftShooterMotor = HWMap.get(DcMotor.class, "leftShooter");
        rightShooterMotor = HWMap.get(DcMotor.class, "rightShooter");

        leftShooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightShooterMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        leftShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightShooterMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public static double calculatePower(double distance) {
        // TODO: set launch angle
        double angle = Math.toRadians(55);
        // TODO: set target height (meters)
        double targetHeight = 20;
        // TODO: set wheel radius (meters)
        double wheelRadius = 0.5;
        // TODO: set motor RPM
        double motorMaxRPM = 6000;

        double numerator = 9.81 * distance * distance;
        double denominator = 2 * Math.cos(angle) * Math.cos(angle) * (distance * Math.tan(angle) - targetHeight);

        if (denominator <= 0) return 0;

        double vIdeal = Math.sqrt(numerator / denominator);

        double k = 0.1; // experimental drag constant for balls
        double vEff = vIdeal * (1 + k * distance);

        double wheelRPM = (60.0 / (2 * Math.PI * wheelRadius)) * vEff;

        double power = wheelRPM / motorMaxRPM;
        return Math.min(power, 1.0);
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

            return time.milliseconds() < (RECOVERY_CONSTANT / power) * 1000;
        }
    }
    public Action powerUp(double d) {
        return new PowerUp(d);
    }
    public Action powerUp(Distances d) {
        return new PowerUp(d);
    }
}
