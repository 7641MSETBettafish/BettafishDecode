package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.SleepAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.mechanisms.Camera;


@Config
@Autonomous(preselectTeleOp = "ABlueTeleop")
public class Auto_pathing_close_side extends LinearOpMode {

    Camera camera;


    @Override
    public void runOpMode() {

        Pose2d startPose1 = new Pose2d(-52, -50, Math.toRadians(54));
        Pose2d startPose2 = new Pose2d(-24, -16, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose1);

        camera = new Camera(hardwareMap);

        TrajectoryActionBuilder preload = drive.actionBuilder(startPose1)
                .strafeToLinearHeading(new Vector2d(-36, -36), Math.toRadians(45))
                .waitSeconds(1)// launch preload
                .strafeToLinearHeading(new Vector2d(-35, -35), Math.toRadians(-20))
                .waitSeconds(2); //detect motif


        TrajectoryActionBuilder path1 = drive.actionBuilder(startPose2) // green purple purple, only for specific cases
                .splineToLinearHeading(new Pose2d(36, -30, Math.toRadians(-90)), Math.toRadians(-90))
                .waitSeconds(0.5)
                .lineToY(-52) //intake balls
                .setTangent(20)
                .splineToLinearHeading(new Pose2d(-45, -30, Math.toRadians(45)), Math.toRadians(-90)); //launch balls

        TrajectoryActionBuilder path2 = drive.actionBuilder(startPose2) //purple green purple path
                .splineToLinearHeading(new Pose2d(12.5, -30, Math.toRadians(-90)), Math.toRadians(-90))
                .waitSeconds(0.5)
                .lineToY(-52) //intake balls
                .setTangent(20)
                .splineToLinearHeading(new Pose2d(-45, -30, Math.toRadians(45)), Math.toRadians(-90)); //launch balls

        TrajectoryActionBuilder path3 = drive.actionBuilder(startPose2) //purple purple green path, preload then intake then go back and launch
                .splineToLinearHeading(new Pose2d(-11, -30, Math.toRadians(-90)), Math.toRadians(-90))
                .waitSeconds(0.5)
                .lineToY(-52) //intake balls
                .strafeToLinearHeading(new Vector2d(-45, -30), 45); //launch balls


        Action preload1 = preload.build();

        Action path21 = path1.build();

        Action path22 = path2.build();

        Action path23 = path3.build();


        waitForStart();

        Actions.runBlocking(new SequentialAction(
                preload1, // actions
                camera.findID(),
                new SleepAction(0.1)
        ));


        try {
            if (camera.id == 21) {
                telemetry.addData("id", camera.id);
                telemetry.update();
                Actions.runBlocking(new ParallelAction(
                        //actions
                        path21

                ));
            } else if (camera.id == 22) {
                telemetry.addData("id", camera.id);
                telemetry.update();
                Actions.runBlocking(new ParallelAction(
                        //actions
                        path22
                ));

            } else if (camera.id == 23) {
                telemetry.addData("id", camera.id);
                telemetry.update();
                Actions.runBlocking(new ParallelAction(
                        //actions
                        path23
                ));

            } else {
                Actions.runBlocking(new ParallelAction(
                        path23
                ));
            }
        } catch (NullPointerException e) {
            Actions.runBlocking(new ParallelAction(
                    path23
            ));
        }
    }
}
