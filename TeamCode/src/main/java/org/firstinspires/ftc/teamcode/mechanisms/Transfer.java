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
    public static int transferticks = 1000;     // encoder ticks to run after detection
    public static double detectionDistance = 5; // cm i think

    private DcMotor transferMotor;
    private DistanceSensor transferSensor;

    public Transfer(HardwareMap hwMap) {
        transferMotor = hwMap.get(DcMotor.class, "transfer");
        transferMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        transferSensor = hwMap.get(DistanceSensor.class, "transferSensor");
    }

    public class Run implements Action {
        private boolean detected = false;
        private int startPos = 0;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            double dist = transferSensor.getDistance(org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit.CM);
            packet.put("transferDist", dist);
            packet.put("detected", detected);

            if (!detected) {
                transferMotor.setPower(transferPower);
                if (dist < detectionDistance) {
                    detected = true;
                    startPos = transferMotor.getCurrentPosition();
                }
            } else {

                transferMotor.setPower(transferPower);
                int delta = Math.abs(transferMotor.getCurrentPosition() - startPos);
                if (delta >= transferticks) {
                    transferMotor.setPower(0);
                    return false;
                }
            }
            return true;
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
