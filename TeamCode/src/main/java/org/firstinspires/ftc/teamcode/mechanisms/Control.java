package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;

public class Control {
    private boolean busy = false;
    private boolean finished = false;


    public void resetFinished() {
        finished = false;
    }

    public void resetBusy() {
        busy = false;
    }

    public boolean isFinished() {
        return finished;
    }

    public boolean isBusy() {
        return busy;
    }

    public class Start implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            busy = true;
            finished = false;
            return false;
        }
    }
    public Action start() {
        return new Start();
    }

    public class End implements Action {

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            busy = false;
            finished = true;
            return false;
        }
    }
    public Action end() {
        return new End();
    }

}
