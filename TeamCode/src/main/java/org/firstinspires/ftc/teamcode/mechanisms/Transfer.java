package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Config
public class Transfer {
    public static double transferPower = 1;
    public static double detectionDistance = 5;

    public DcMotor transferMotor;
    public DistanceSensor transferSensor;

    public Transfer(HardwareMap hwMap) {
        transferMotor = hwMap.get(DcMotor.class, "transfer");

        transferMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        transferSensor = hwMap.get(DistanceSensor.class, "transferSensor");
    }

    public class Run implements Action {

        boolean init = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!init) {
                transferMotor.setPower(transferPower);
                init = true;
            }

            if (transferSensor.getDistance(DistanceUnit.CM) <= detectionDistance) {
                transferMotor.setPower(0);
                return false;
            } else {
                return true;
            }
        }
    }
    public Action run() {
        return new Run();
    }

    public class Load implements Action {

        boolean init = false;
        ElapsedTime time = new ElapsedTime();

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!init) {
                transferMotor.setPower(transferPower);
                time.reset();
            }

            if ((transferSensor.getDistance(DistanceUnit.CM) <= detectionDistance && time.milliseconds() > 750) || time.milliseconds() > 1250) {
                transferMotor.setPower(0);
                return false;
            } else {
                return true;
            }
        }
    }
    public Action load() {
        return new Load();
    }
}
