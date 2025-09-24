package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(100, 50, Math.toRadians(180), Math.toRadians(180), 16.5)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(60, -16, 180))
                .strafeToLinearHeading(new Vector2d(-16, -16), 45)
                .waitSeconds(1) //launch balls
                .splineToLinearHeading(new Pose2d(36, -30, Math.toRadians(-90)), Math.toRadians(-90))
                .waitSeconds(0.5)
                .lineToY(-52) //intake balls
                .setTangent(20)
                .splineToLinearHeading(new Pose2d(-16, -16, Math.toRadians(45)), Math.toRadians(-90)) //launch balls
                .waitSeconds(3) //deposit
                .setTangent(20)
                .splineToLinearHeading(new Pose2d(13.5, -30  , Math.toRadians(-90)), Math.toRadians(-90))
                .lineToY(-52)
                .waitSeconds(1)//intake balls
                .setTangent(20)
                .splineToLinearHeading(new Pose2d(-16, -16, Math.toRadians(45)), Math.toRadians(-90))
                .waitSeconds(3) // deposit
                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_DECODE_JUICE_BLACK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}