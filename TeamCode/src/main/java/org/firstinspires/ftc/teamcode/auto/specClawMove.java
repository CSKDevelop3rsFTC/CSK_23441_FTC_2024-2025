package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Autonomous(name = "specClawMove")
public class specClawMove extends LinearOpMode {
    private Servo specClaw = null;

    public void specMove() {
        specClaw.setPosition(0.5);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        specClaw = hardwareMap.get(Servo.class, "specServo");

        waitForStart();
        
        specMove();

    }
}