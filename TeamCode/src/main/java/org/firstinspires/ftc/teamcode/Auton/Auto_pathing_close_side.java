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
import org.firstinspires.ftc.teamcode.mechanisms.Intake;
import org.firstinspires.ftc.teamcode.mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.mechanisms.Transfer;


@Config
@Autonomous(preselectTeleOp = "ABlueTeleop")
public class Auto_pathing_close_side extends LinearOpMode {

    Camera camera;

    double distance = 0;


    @Override
    public void runOpMode() {

        Pose2d startPose1 = new Pose2d(-52, -50, Math.toRadians(54));
        //Pose2d startPose2 = new Pose2d(-24, -16, Math.toRadians(0));
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose1);

        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);

        camera = new Camera(hardwareMap);

        TrajectoryActionBuilder preload = drive.actionBuilder(startPose1)
                .strafeToLinearHeading(new Vector2d(-36, -36), Math.toRadians(45))
                .waitSeconds(1)// launch preload
                .strafeToLinearHeading(new Vector2d(-35, -35), Math.toRadians(-20))
                .waitSeconds(2); //detect motif


        TrajectoryActionBuilder path1 = preload.fresh() // green purple purple, only for specific cases
                .splineToLinearHeading(new Pose2d(40, -30, Math.toRadians(-90)), Math.toRadians(-90))
                .waitSeconds(0.5)
                .lineToY(-52) //intake balls
                .setTangent(20)
                .splineToLinearHeading(new Pose2d(-36, -36, Math.toRadians(45)), Math.toRadians(-90)); //launch balls

        TrajectoryActionBuilder path2 = preload.fresh() //purple green purple path
                .splineToLinearHeading(new Pose2d(16.5, -30, Math.toRadians(-90)), Math.toRadians(-90))
                .waitSeconds(0.5)
                .lineToY(-52) //intake balls
                .setTangent(20)
                .splineToLinearHeading(new Pose2d(-36, -36, Math.toRadians(45)), Math.toRadians(-90)); //launch balls


        TrajectoryActionBuilder path3 = preload.fresh() //purple purple green path, preload then intake then go back and launch
                .splineToLinearHeading(new Pose2d(-7, -30, Math.toRadians(-90)), Math.toRadians(-90))
                .waitSeconds(0.5)
                .lineToY(-52) //intake balls
                .splineToLinearHeading(new Pose2d(-36, -36, Math.toRadians(45)), Math.toRadians(-90)); //launch balls


        Action preload1 = preload.build();

        Action path21 = path1.build();

        Action path22 = path2.build();

        Action path23 = path3.build();


        waitForStart();

        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        new SequentialAction(
                                new SleepAction(1),
                                shooter.powerUp(distance),
                                transfer.load(),
                                shooter.stop()

                        ),

                        preload1
                ),

                camera.findID()
                // new SleepAction(0.1)
        ));


        try {
            if (camera.id == 21) {
                telemetry.addData("id", camera.id);
                telemetry.update();
                Actions.runBlocking(new ParallelAction(
                        //new SequentialAction(
                                //new ParallelAction(
                                        //intake.run(),
                                        //transfer.load(),
                                        //intake.stop()
                                //),

                                //shooter.powerUp(distance),
                                //transfer.load(),
                                //shooter.stop()

                        //),
                        //actions
                        path21

                ));
            } else if (camera.id == 22) {
                telemetry.addData("id", camera.id);
                telemetry.update();
                Actions.runBlocking(new ParallelAction(
                             //new SequentialAction(
                                     //new ParallelAction(
                                        //intake.run(),
                                        //transfer.load(),
                                        //intake.stop()
                                     //),

                                     //shooter.powerUp(distance),
                                     //transfer.load(),
                                     //shooter.stop()

                                     //),                   //intake start //transfer load // intake stop
                                                             //shooter power up // transfer load //shooter power down



                        path22
                ));

            } else if (camera.id == 23) {
                telemetry.addData("id", camera.id);
                telemetry.update();
                Actions.runBlocking(new ParallelAction(
                        //new SequentialAction(
                                //new ParallelAction(
                                        //intake.run(),
                                        //transfer.load(),
                                        //intake.stop()
                                //),

                                //shooter.powerUp(distance),
                                //transfer.load(),
                                //shooter.stop()

                        //),
                        path23
                ));

            } else {
                Actions.runBlocking(new ParallelAction(
                        //new SequentialAction(
                                //new ParallelAction(
                                        //intake.run(),
                                        //transfer.load(),
                                        //intake.stop()
                                //),

                                //shooter.powerUp(distance),
                                //transfer.load(),
                                //shooter.stop()

                        //),
                        path23
                ));
            }
        } catch (NullPointerException e) {
            Actions.runBlocking(new ParallelAction(
                    //new SequentialAction(
                            //new ParallelAction(
                                    //intake.run(),
                                    //transfer.load(),
                                    //intake.stop()
                            //),

                            //shooter.powerUp(distance),
                            //transfer.load(),
                            //shooter.stop()

                    //),

                    path23
            ));
        }
    }
}
