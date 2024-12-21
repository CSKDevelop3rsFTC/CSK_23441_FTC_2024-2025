package com.example.meepmeeptesting;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class MeepMeepTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(600);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(40, 40, Math.PI, Math.PI, 11.456382173167453)
                .build();

        myBot.runAction(myBot.getDrive().actionBuilder(new Pose2d(-64, -11, Math.toRadians(180)))

                .setTangent(0)
                .splineToConstantHeading(new Vector2d(-41,0), Math.PI / 2) // move up to center rungs

                .strafeTo(new Vector2d(-33, 0)) // go forward to line up specimen

                .waitSeconds(1.25)// replace later with action of placing specimen

                .strafeTo(new Vector2d(-45, 0)) // move backwards a bit

                .splineTo(new Vector2d(-52,-33 ), 0)

                .waitSeconds(1)

                .setTangent(Math.PI)
                .splineTo(new Vector2d(-31,0), 0)


                /*
                .setTangent(0)
                .splineToConstantHeading(new Vector2d(0,-41), Math.PI / 2) // move up to center rungs

                .strafeTo(new Vector2d(0, -33)) // go forward to line up specimen

                .waitSeconds(1.25) // replace later with action of placing specimen

                .strafeTo(new Vector2d(0, -45)) // move backwards a bit

                .splineTo(new Vector2d(33, -56), Math.PI / 2)

                .waitSeconds(1)

                .splineTo(new Vector2d(0,-33), Math.PI / 2) // move up to center rungs

                .strafeTo(new Vector2d(0, -43)) // move backwards a bit

                .splineTo(new Vector2d(33, -56), Math.PI / 2)

                .waitSeconds(1)

                .splineTo(new Vector2d(0,-33), Math.PI / 2) // move up to center rungs

                .strafeTo(new Vector2d(0, -43)) // move backwards a bit

                .splineTo(new Vector2d(33, -56), Math.PI / 2)

                .waitSeconds(1)

                .splineTo(new Vector2d(0,-33), Math.PI / 2) // move up to center rungs

                 */

                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_INTO_THE_DEEP_JUICE_DARK)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}