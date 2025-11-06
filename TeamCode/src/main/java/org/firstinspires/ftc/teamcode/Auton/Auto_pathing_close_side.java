package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.*;
import org.firstinspires.ftc.teamcode.mechanisms.*;


@Config
@Autonomous(preselectTeleOp = "ABlueTeleop")
public class Auto_pathing_close_side extends LinearOpMode {

    Camera camera;

    public void runOpMode() {

        Pose2d startPose1 = new Pose2d(-52, -50, Math.toRadians(54));

        Shooter shooter = new Shooter(hardwareMap);

        Intake intake = new Intake(hardwareMap);

        Transfer transfer = new Transfer(hardwareMap, shooter);

        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose1);


        camera = new Camera(hardwareMap);
        TrajectoryActionBuilder preload = drive.actionBuilder(startPose1)
                .strafeToLinearHeading(new Vector2d(-10, -10), Math.toRadians(0));


        TrajectoryActionBuilder path2 = drive.actionBuilder(new Pose2d(-10, -10, Math.toRadians(0))) // green purple purple, only for specific cases
                .strafeToLinearHeading(new Vector2d(-25, -25), Math.toRadians(45))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(12, -30), Math.toRadians(-90))
                .waitSeconds(1)
                .lineToY(-50)
                .waitSeconds(0.2)
                .lineToY(-30)
                .waitSeconds(0.1)
                .strafeToLinearHeading(new Vector2d(-25, -25), Math.toRadians(45))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-12, -30), Math.toRadians(-90))
                .waitSeconds(1)
                .lineToY(-50)
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-25, -25), Math.toRadians(45));


        TrajectoryActionBuilder path1 = drive.actionBuilder(new Pose2d(-10, -10, Math.toRadians(0))) // purple purple green, only for specific cases
                .strafeToLinearHeading(new Vector2d(-25, -25), Math.toRadians(45))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-12, -30), Math.toRadians(-90))
                .waitSeconds(1)
                .lineToY(-50)
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-25, -25), Math.toRadians(45))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(12, -30), Math.toRadians(-90))
                .waitSeconds(1)
                .lineToY(-50)
                .strafeToLinearHeading(new Vector2d(-25, -25), Math.toRadians(45));

        Action path21 = path1.build();


        Action path22 = path2.build();

        Action preload1 = preload.build();


        waitForStart();

        Actions.runBlocking(new SequentialAction(

                camera.findID(),
                new ParallelAction(
                        intake.run(),
                        transfer.fullLoad(),
                        intake.stop(),
                        preload1
                )
        ));


        if (camera.id == 21 || camera.id == 23) {
            telemetry.addData("id", camera.id);
            telemetry.update();
            Actions.runBlocking(new ParallelAction(
                    new SequentialAction(
                            new ParallelAction(
                                    intake.run(),
                                    transfer.fullLoad(),
                                    intake.stop()
                            ),

                            transfer.fullLoad(),
                            shooter.stop(),

                            path21

                    )));
        } else if (camera.id == 22) {
            telemetry.addData("id", camera.id);
            telemetry.update();
            Actions.runBlocking(new ParallelAction(
                    new SequentialAction(
                            new ParallelAction(
                                    intake.run(),
                                    transfer.fullLoad(),
                                    intake.stop()
                            ),

                            transfer.fullLoad(),
                            shooter.stop(),

                            path22
                    )));

        } else {
            Actions.runBlocking(new ParallelAction(
                    new SequentialAction(
                            new ParallelAction(
                                    intake.run(),
                                    transfer.fullLoad(),
                                    intake.stop()
                            ),

                            transfer.fullLoad(),
                            shooter.stop(),

                            path21
                    )));
        }
    }
}
