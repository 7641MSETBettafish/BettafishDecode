package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;


@Config
public class Transfer {
    public static double transferPower = 1;
    public static double detectionDistance = 5; // cm i think

    private DcMotor transferMotor;
    private DistanceSensor transferSensor;

    public Transfer(HardwareMap hwMap) {
        transferMotor = hwMap.get(DcMotor.class, "transfer");


        transferSensor = hwMap.get(DistanceSensor.class, "transferSensor");
    }

    public class Run implements Action {
        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double dist = transferSensor.getDistance(DistanceUnit.CM);
            packet.put("transferDist", dist);

            if (dist <= detectionDistance) {
                transferMotor.setPower(0);
                return false;
            } else {
                transferMotor.setPower(transferPower);
                return true;
            }
        }
    }

    public class deposit implements Action {
        public boolean run(@NonNull TelemetryPacket packet) {
            transferMotor.setPower(transferPower);
            return false;
        }
    }

    public class outtake implements Action {
        public boolean run(@NonNull TelemetryPacket packet) {
            transferMotor.setPower(-transferPower);
            return false;
        }
    }


    public class Stop implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            transferMotor.setPower(0);
            return false;
        }

    }
    public Action stop() {
        return new Transfer.Stop();
    }
}
