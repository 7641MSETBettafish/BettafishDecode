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
import org.firstinspires.ftc.teamcode.mechanisms.Context;
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Transfer;


@Config
@Autonomous(preselectTeleOp = "Teleop")
public class Auton_Blue_nine_ball extends LinearOpMode {

    Camera camera;


    @Override
    public void runOpMode() {

        Pose2d startPose1 = new Pose2d(-52, -50, Math.toRadians(54));

        Shooter shooter = new Shooter(hardwareMap);

        Transfer transfer = new Transfer(hardwareMap);
        Intake intake = new Intake(hardwareMap);



        Context context = new Context();
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose1);

        camera = new Camera(hardwareMap);

        TrajectoryActionBuilder path0 =  drive.actionBuilder(startPose1)
                .strafeToLinearHeading(new Vector2d(-30, -20), Math.toRadians(45))
                .waitSeconds(0.85)
                .turnTo(0.729727656)
                .waitSeconds(1);

        TrajectoryActionBuilder path1 =  drive.actionBuilder(new Pose2d(-30,-20, 0.729727656))
                .strafeToLinearHeading(new Vector2d(-37, -20), Math.toRadians(-90))
                .waitSeconds(0.01)
                .lineToY(-45)
                .waitSeconds(0.01)
                .strafeToLinearHeading(new Vector2d(-25, -48), Math.toRadians(-150))
                .waitSeconds(0.01)
                .splineToLinearHeading(new Pose2d(-30, -20, Math.toRadians(45)), Math.toRadians(45))
                .waitSeconds(0.8)
                .strafeToLinearHeading(new Vector2d(-9, -20), Math.toRadians(-90))
                .waitSeconds(0.01)
                .lineToY(-51)
                .waitSeconds(0.01)
                .strafeToLinearHeading(new Vector2d(-30, -20), Math.toRadians(45))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(10, -20), Math.toRadians(-90))
                .waitSeconds(0.01)
                .lineToY(-53)
                .waitSeconds(0.01)
                .strafeToLinearHeading(new Vector2d(-30, -20), Math.toRadians(45))
                .waitSeconds(1.5)
                .strafeToLinearHeading(new Vector2d(0, -30), Math.toRadians(0));

        TrajectoryActionBuilder path23 =  drive.actionBuilder(new Pose2d(-30,-20, 0.729727656))
                .strafeToLinearHeading(new Vector2d(-37, -20), Math.toRadians(-90))
                .waitSeconds(0.01)
                .lineToY(-45)
                .waitSeconds(0.01)
                .strafeToLinearHeading(new Vector2d(-25, -48), Math.toRadians(-150))
                .waitSeconds(0.01)
                .strafeToLinearHeading(new Vector2d(-30, -20), Math.toRadians(45))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(10, -20), Math.toRadians(-90))
                .waitSeconds(0.01)
                .lineToY(-53)
                .waitSeconds(0.01)
                .strafeToLinearHeading(new Vector2d(-30, -20), Math.toRadians(45))
                .waitSeconds(1.5)
                .splineToLinearHeading(new Pose2d(-30, -20, Math.toRadians(45)), Math.toRadians(45))
                .waitSeconds(0.8)
                .strafeToLinearHeading(new Vector2d(-9, -20), Math.toRadians(-90))
                .waitSeconds(0.01)
                .lineToY(-51)
                .waitSeconds(0.01)
                .strafeToLinearHeading(new Vector2d(0, -30), Math.toRadians(0));

        Action Prepath = path0.build();

        Action path2 = path1.build();

        Action pathfar = path23.build();

        waitForStart();



        Actions.runBlocking(new SequentialAction(
                camera.findID(),
                new ParallelAction(
                    new SequentialAction(
                            intake.run(),
                            new SleepAction(1.2),
                            transfer.fullLoad()
                    ),
                    Prepath
                )

        ));

        if (camera.id == 23) {
            Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        shooter.run(3120),
                        Context.updatePosition(drive, 0),
                        new SequentialAction(
                                new SleepAction(2.4),
                                transfer.run(),
                                new SleepAction(4.1),
                                transfer.fullLoad(),
                                new SleepAction(2.0),
                                transfer.run(),
                                new SleepAction(2.6),
                                transfer.fullLoad(),
                                new SleepAction(2.5),
                                transfer.run(),
                                new SleepAction(3.2),
                                transfer.fullLoad()
                        ),

                        pathfar
                )
            ));
        } else {
            Actions.runBlocking(new SequentialAction(
                    new ParallelAction(
                            shooter.run(3120),
                            Context.updatePosition(drive, 0),
                            new SequentialAction(
                                    new SleepAction(2.4),
                                    transfer.run(),
                                    new SleepAction(4.1),
                                    transfer.fullLoad(),
                                    new SleepAction(2.0),
                                    transfer.run(),
                                    new SleepAction(2.6),
                                    transfer.fullLoad(),
                                    new SleepAction(2.5),
                                    transfer.run(),
                                    new SleepAction(3.2),
                                    transfer.fullLoad()

                            ),

                            path2
                    )
            ));
        }



    }

}
