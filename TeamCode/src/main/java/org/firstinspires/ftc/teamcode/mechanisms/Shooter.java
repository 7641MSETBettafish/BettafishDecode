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
    double flywheelInertia = 0.5 * 1 * 2 * 2;
    // TODO: set max motor speed at 12V
    double motorMaxRPM = 6000;
    // TODO: how much speed is conserved
    double gearEfficiency = 1;
    // TODO: gear ratio is motor speed / flywheel speed
    double gearRatio = 3;
    // TODO: set to motor torque (at 12V when stalled)
    double motorStallTorque = 0.35;
    double RECOVERY_CONSTANT = (flywheelInertia * motorMaxRPM * 2 * Math.PI / 60.0) / (gearEfficiency * gearRatio * gearRatio * motorStallTorque);

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

    public DcMotor shooterMotor1;
    public DcMotor shooterMotor2;

    public Shooter(HardwareMap HWMap) {
        shooterMotor1 = HWMap.get(DcMotor.class, "shooter1");
        shooterMotor2 = HWMap.get(DcMotor.class, "shooter2");
    }

    public static double calculatePower(double distance) {
        // TODO: set launch angle
        double angle = Math.toRadians(0);
        // TODO: set target height (meters)
        double targetHeight = 20;
        // TODO: set wheel radius (meters)
        double wheelRadius = 0.5;
        // TODO: set motor RPM
        double motorMaxRPM = 6000;

        double numerator = 9.81 * distance * distance;
        double denominator = 2 * Math.cos(angle) * Math.cos(angle) * (distance * Math.tan(angle) - targetHeight);

        if (denominator <= 0) return 1; // impossible shot at given angle so just hail mary :p

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
            double power = calculatePower(distance);

            if (!init) {
                shooterMotor1.setPower(power);
                shooterMotor2.setPower(power);
                time.reset();
                init = true;
            }

            return time.milliseconds() < (RECOVERY_CONSTANT / power) * 1000;
        }
    }
}
