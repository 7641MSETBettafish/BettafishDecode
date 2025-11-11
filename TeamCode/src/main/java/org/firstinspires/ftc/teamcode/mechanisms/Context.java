package org.firstinspires.ftc.teamcode.mechanisms;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.*;
import org.firstinspires.ftc.teamcode.teleop.Teleop;

public class Context {
    public static void setContext(double robotx, double roboty, double roboth, double goalside) {
        Teleop.startX = robotx;
        Teleop.startY = roboty;
        Teleop.startH = roboth;
        Teleop.goalSide = goalside;
    }
    public static class UpdatePosition implements Action {
        MecanumDrive drive;
        int goal;

        public UpdatePosition(MecanumDrive drive, int g) {
            this.drive = drive;
            this.goal = g;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            setContext(drive.localizer.getPose().position.x, drive.localizer.getPose().position.y, drive.localizer.getPose().heading.toDouble(), goal);
            return true;
        }
    }

    public static Action updatePosition(MecanumDrive drive, int goal) {
        return new UpdatePosition(drive, goal);
    }
}
