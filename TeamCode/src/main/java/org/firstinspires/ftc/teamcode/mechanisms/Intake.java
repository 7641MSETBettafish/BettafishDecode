package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Intake {

    public static double intakePower = 0.85;

    public static double detectionDistance = 6.5;

    public DcMotor intakeMotor;
    //public DistanceSensor leftIntakeSensor;
    //public DistanceSensor rightIntakeSensor;

    public Intake(HardwareMap HWMap) {
        intakeMotor = HWMap.get(DcMotor.class, "intake");
        intakeMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        //leftIntakeSensor = HWMap.get(DistanceSensor.class, "leftIntakeSensor");
        //rightIntakeSensor = HWMap.get(DistanceSensor.class, "rightIntakeSensor");

    }

    public double getDistance() {
        //return Math.min(leftIntakeSensor.getDistance(DistanceUnit.CM), rightIntakeSensor.getDistance(DistanceUnit.CM));
        return -67;
    }

    public boolean ballSensed() {
        return getDistance() <= detectionDistance;
    }

    public class Run implements Action {

        boolean init = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!init) {
                intakeMotor.setPower(intakePower);
                init = true;
            }

            if (ballSensed()) {
                intakeMotor.setPower(0);
                return false;
            } else {
                return true;
            }

        }

    }

    public Action run() {
        return new Run();
    }

    public class Stop implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            intakeMotor.setPower(0);
            return false;
        }

    }
    public Action stop() {
        return new Stop();
    }
}
