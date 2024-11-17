package org.firstinspires.ftc.teamcode.auto;

import androidx.annotation.NonNull;

import org.firstinspires.ftc.teamcode.teleop.Meet0TeleOp;
// RR-specific imports
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.ParallelAction;

// Non-RR imports
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.PinpointDrive;

@Autonomous(name = "autoTest", group = "Autonomous")
public class autoTest extends LinearOpMode {

    public void intakething(){
        autoBasic auto = new autoBasic();
        auto.hzFourBar(1);
        auto.spinTake(1,300);
        auto.hzFourBar(0);
    }


public class Intake{
    public class IntakeSample implements Action {
        private boolean initialized = false;
        autoBasic auto = new autoBasic();

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                auto.hzFourBar(1);
                auto.spinTake(1, 300);
            }

            double pos1 = auto.hzfourbarServo1.getPosition();
            double pos2 = auto.hzfourbarServo2.getPosition();
            if (pos1 == 0 && pos2 == 0) {
                return true;
            } else {
                auto.intakeServo.setPower(0);
                return false;
            }
        }
    }

    public Action intakeSample() {
        return new IntakeSample();
    }

    public class IntakeBack implements Action {
        private boolean initialized = false;
        autoBasic auto = new autoBasic();

        @Override
        public boolean run(@NonNull TelemetryPacket telemetryPacket) {
            if (!initialized) {
                auto.hzFourBar(0);
                auto.intakeServo.setPower(0);
            }

            double pos1 = auto.hzfourbarServo1.getPosition();
            double pos2 = auto.hzfourbarServo2.getPosition();
            if (pos1 == 0.95 && pos2 == 0.95) {
                return true;
            } else {
                auto.spinTake(1, 300);
                return false;
            }
        }
    }

    public Action intakeBack() {
        return new IntakeBack();
    }
}
    public class Claw {
        public class OpenClaw implements Action {
            autoBasic auto = new autoBasic();

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                    auto.clawPosition(1);
                    return false;
            }
        }
        public Action openClaw(){
            return new OpenClaw();
        }

        public class CloseClaw implements Action {
            autoBasic auto = new autoBasic();

            @Override
            public boolean run(@NonNull TelemetryPacket telemetryPacket) {
                auto.clawPosition(0);
                return false;
            }
        }
        public Action closeClaw(){
            return new CloseClaw();
        }
    }


    public void runOpMode() {
        Intake intake = new Intake();
        Claw claw = new Claw();
        autoBasic auto = new autoBasic();

        // instantiate your MecanumDrive at a particular pose.
        Pose2d beginPose = new Pose2d(-63, -11, 0);
        PinpointDrive drive = new PinpointDrive(hardwareMap, beginPose);

        waitForStart();

        TrajectoryActionBuilder tab1 = drive.actionBuilder(beginPose)
                        .setTangent(0)
                        .splineToConstantHeading(new Vector2d(-41,0), Math.PI / 2) // move up to center rungs
                        .strafeTo(new Vector2d(-33, 0)) // go forward to line up specimen
                        .waitSeconds(3);
        Action trajectoryActionCloseOut = tab1.fresh()
                        .strafeTo(new Vector2d(-45, 0)) // move backwards a bit
                        .build();

        Action trajectoryActionChosen;
        trajectoryActionChosen = tab1.build();
        //Action trajectoryActionChosen2 = tab2.build();
        Actions.runBlocking(
                            new SequentialAction(
                                    trajectoryActionChosen,
                                    intake.intakeSample(),
                                    intake.intakeBack(),
                                    trajectoryActionCloseOut

                            )
                    );


    }

}
