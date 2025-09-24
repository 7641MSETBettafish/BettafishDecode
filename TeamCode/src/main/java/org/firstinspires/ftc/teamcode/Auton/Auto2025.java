package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
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

    public Integer tagID;
    Camera camera;


    @Override
    public void runOpMode() {

        Pose2d startPose1 = new Pose2d(-58, -55, Math.toRadians(45));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose1);

        camera = new Camera(hardwareMap);


        TrajectoryActionBuilder path1 = drive.actionBuilder(startPose1) // green purple purple, only for specific cases
                .strafeToLinearHeading(new Vector2d(-45, -30), 45)
                .waitSeconds(1) //launch balls
                .splineToLinearHeading(new Pose2d(36, -30, Math.toRadians(-90)), Math.toRadians(-90))
                .waitSeconds(0.5)
                .lineToY(-52) //intake balls
                .setTangent(20)
                .splineToLinearHeading(new Pose2d(-45, -30, Math.toRadians(45)), Math.toRadians(-90)); //launch balls

        TrajectoryActionBuilder path2 = drive.actionBuilder(startPose1) //purple green purple path
                .strafeToLinearHeading(new Vector2d(-45, -30), 45)
                .waitSeconds(1) //launch balls
                .splineToLinearHeading(new Pose2d(12.5, -30, Math.toRadians(-90)), Math.toRadians(-90))
                .waitSeconds(0.5)
                .lineToY(-52) //intake balls
                .setTangent(20)
                .splineToLinearHeading(new Pose2d(-45, -30, Math.toRadians(45)), Math.toRadians(-90)); //launch balls

        TrajectoryActionBuilder path3 = drive.actionBuilder(startPose1) //purple purple green path, preload then intake then go back and launch
                .strafeToLinearHeading(new Vector2d(-45, -30), 45)
                .waitSeconds(1) //launch balls
                .splineToLinearHeading(new Pose2d(-11, -30, Math.toRadians(-90)), Math.toRadians(-90))
                .waitSeconds(0.5)
                .lineToY(-52) //intake balls
                .strafeToLinearHeading(new Vector2d(-45, -30), 45); //launch balls

        TrajectoryActionBuilder path4 = drive.actionBuilder(startPose1)
                .waitSeconds(5)
                .strafeToConstantHeading(new Vector2d(24, 60));

        Action path21 = path1.build();

        Action path22 = path2.build();

        Action path23 = path3.build();

        Action pathOther = path4.build();

        waitForStart();

        if (tagID == 21) {
            telemetry.addData("id", tagID);
            telemetry.update();
            Actions.runBlocking(new ParallelAction(
                    path21,
                    camera.findID(tagID)
            ));
        } else if (tagID == 22) {
            telemetry.addData("id", tagID);
            telemetry.update();
            Actions.runBlocking(new ParallelAction(
                    path22,
                    camera.findID(tagID)
            ));

        } else if (tagID == 23) {
            telemetry.addData("id", tagID);
            telemetry.update();
            Actions.runBlocking(new ParallelAction(
                    path23,
                    camera.findID(tagID)
            ));

        } else {
            telemetry.addData("id", tagID);
            telemetry.update();
            Actions.runBlocking(new ParallelAction(
                    path23,
                    camera.findID(tagID)
            ));
            Actions.runBlocking(pathOther);
        }
    }
}
