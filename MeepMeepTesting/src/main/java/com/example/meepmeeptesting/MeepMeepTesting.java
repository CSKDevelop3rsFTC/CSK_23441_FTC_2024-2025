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
                .setConstraints(40, 40, Math.PI, Math.PI, 11.456382173167453)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-64, -11, 0))
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(-41,0), Math.PI / 2)

                .strafeTo(new Vector2d(-33, 0))

                .waitSeconds(3)

                .setTangent(Math.PI / 2)
                .setReversed(true)
                .splineToConstantHeading(new Vector2d(-44.8, 59), Math.PI / 2)

                .turnTo(-12)

                .waitSeconds(3)

                .turnTo(11)

                .strafeTo(new Vector2d(-56.9, 57))

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}