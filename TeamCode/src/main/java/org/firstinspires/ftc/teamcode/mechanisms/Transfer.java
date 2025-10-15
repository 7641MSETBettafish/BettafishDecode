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
    //how fast we want the transfer to run when we run it
    public static double transferPower = 0.7;
    //how close the distance sensor needs to sense for it to stop the transfer
    public static double detectionDistance = 3;
    //how many ticks it takes for the transfer to push the ball into the shooter and bring the next ball into shooting posiition
    public static double loadDistance = 100;

    public DcMotor transferMotor;
    public DistanceSensor leftTransferSensor;
    public DistanceSensor rightTransferSensor;

    public Transfer(HardwareMap hwMap) {
        //initialize transfer motor
        transferMotor = hwMap.get(DcMotor.class, "transfer");

        //reset 
        transferMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        leftTransferSensor = hwMap.get(DistanceSensor.class, "leftTransferSensor");
        rightTransferSensor = hwMap.get(DistanceSensor.class, "rightTransferSensor");
    }

    public double getDistance() {
        return Math.min(leftTransferSensor.getDistance(DistanceUnit.CM), rightTransferSensor.getDistance(DistanceUnit.CM));
    }

    public class Run implements Action {

        boolean init = false;

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!init) {
                transferMotor.setPower(transferPower);
                init = true;
            }

            if (getDistance() <= detectionDistance) {
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

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!init) {
                transferMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                transferMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                transferMotor.setPower(transferPower);
            }

            return transferMotor.getCurrentPosition() <= loadDistance;
        }
    }
    public Action load() {
        return new Load();
    }
}
