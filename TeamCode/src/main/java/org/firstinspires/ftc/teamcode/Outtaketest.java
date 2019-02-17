package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

/*
7890 Space Lions 2019 "Tele Op For Big Kids"
author: 7890 Software (Akira, Erin, Stephen, Kyra, Anthony)
GOALS: 2019, deposite silver minerals, possibly also gold, lower and raise on the lander
 */

@TeleOp(name="outtake test", group="Tele Op")
public class Outtaketest extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    Servo outtake;

    @Override
    public void init() {
        outtake = hardwareMap.servo.get("outtake");
    }

    @Override
    public void loop() {

        double stick = gamepad1.right_stick_x;
        outtake.setPosition(stick);
        telemetry.addData("servopos", stick);
        telemetry.update();
    }
}
