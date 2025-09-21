package org.firstinspires.ftc.teamcode.Auton;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.Vision.Vision;


@Config
@Autonomous(preselectTeleOp = "ABlueTeleop")
public class Auto2025 extends LinearOpMode {

    Camera camera;


    @Override
    public void runOpMode() {

        Pose2d startPose1 = new Pose2d(24, 12, Math.PI / 2);
        MecanumDrive drive = new MecanumDrive(hardwareMap, startPose1);

        camera = new Camera();


        TrajectoryActionBuilder path1 = drive.actionBuilder(startPose1)
                .strafeToConstantHeading(new Vector2d(72, 12));

        Action path = path1.build();

        waitForStart();

        int detected = camera.findID();

        if (detected == 21) {
            // code
        } else if (detected == 22) {
            // code
        } else if (detected == 23) {
            // code
        } else if (detected == 24) {
            // code
        }
    }
}
