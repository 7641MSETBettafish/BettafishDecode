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
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose1);

        Shooter shooter = new Shooter(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Transfer transfer = new Transfer(hardwareMap);

        camera = new Camera(hardwareMap);



        TrajectoryActionBuilder path3 = drive.actionBuilder(startPose1)//purple purple green path, preload then intake then go back and launch
                .strafeToLinearHeading(new Vector2d(-10, -10), Math.toRadians(45))
                .strafeToLinearHeading(new Vector2d(36, -30), Math.toRadians(-90))
                .waitSeconds(1)
                .lineToY(-50)
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-25, -25), Math.toRadians(45))
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(36, -30), Math.toRadians(-90))
                .waitSeconds(1)
                .lineToY(-50)
                .waitSeconds(1)
                .strafeToLinearHeading(new Vector2d(-25, -25), Math.toRadians(45));


        Action path23 = path3.build();


        waitForStart();


        Actions.runBlocking(new SequentialAction(
                new ParallelAction(
                        //intake.run(),
                        //transfer.load(),
                        //intake.stop(),
                        path23
                )

        ));
    }
}
