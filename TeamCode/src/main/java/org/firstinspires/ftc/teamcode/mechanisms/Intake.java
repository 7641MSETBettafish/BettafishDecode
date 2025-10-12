package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Intake {

    public static double intakePower;

    public DcMotor intakeMotor;
    public NormalizedColorSensor colorSensor;

    public Intake(HardwareMap HWMap) {
        intakeMotor = HWMap.get(DcMotor.class, "intake");
        colorSensor = HWMap.get(NormalizedColorSensor.class, "intakeSensor");
        if (!(colorSensor instanceof DistanceSensor)) {
            colorSensor = null;
        }
    }

    public boolean ballSensed() {
        return ((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM) <= 3;
    }

    public class Run implements Action {

        boolean init = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!init) {
                intakeMotor.setPower(intakePower);
                init = true;
            }

            if (((DistanceSensor) colorSensor).getDistance(DistanceUnit.CM) <= 3) {
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
