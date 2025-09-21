package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Vision.Vision;
import org.firstinspires.ftc.teamcode.mechanisms.Camera;


@Config
@Autonomous(preselectTeleOp = "ABlueTeleop")
public class Auto2025 extends LinearOpMode {

    public static int tagID;
    Camera camera;


    @Override
    public void runOpMode() {

        Pose2d startPose1 = new Pose2d(24, 12, Math.PI / 2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose1);

        camera = new Camera(hardwareMap);


        TrajectoryActionBuilder path1 = drive.actionBuilder(startPose1)
                .strafeToConstantHeading(new Vector2d(24, 24));

        TrajectoryActionBuilder path2 = drive.actionBuilder(startPose1)
                .strafeToConstantHeading(new Vector2d(24, 36));

        TrajectoryActionBuilder path3 = drive.actionBuilder(startPose1)
                .strafeToConstantHeading(new Vector2d(24, 48));

        TrajectoryActionBuilder path4 = drive.actionBuilder(startPose1)
                .waitSeconds(5)
                .strafeToConstantHeading(new Vector2d(24, 60));

        Action path21 = path1.build();

        Action path22 = path2.build();

        Action path23 = path3.build();

        Action pathOther = path4.build();

        waitForStart();

        int detected = camera.findID();



        if (detected == 21) {
            telemetry.addData("id", detected);
            telemetry.update();
            Actions.runBlocking(path21);
        } else if (detected == 22) {
            telemetry.addData("id", detected);
            telemetry.update();
            Actions.runBlocking(path22);
        } else if (detected == 23) {
            telemetry.addData("id", detected);
            telemetry.update();
            Actions.runBlocking(path23);
        } else {
            telemetry.addData("id", detected);
            telemetry.update();
            Actions.runBlocking(pathOther);
        }
    }
}
