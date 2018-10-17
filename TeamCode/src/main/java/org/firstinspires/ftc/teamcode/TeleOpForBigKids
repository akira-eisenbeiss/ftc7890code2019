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
author: 7890 Software (Akira, Erin + add names)
GOALS: depositing two glyphs at a time, balancing stone
 */

@TeleOp(name="TeleOp For Big Kids", group="Tele Op")
public class TeleOpForBigKids extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //MOTORS
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor liftMotor;
    DcMotor armMotor;

    //DIRECTIONS
    private DcMotor.Direction LEFTDIRECTION = DcMotor.Direction.REVERSE;
    private DcMotor.Direction RIGHTDIRECTION = DcMotor.Direction.FORWARD;

    //SERVOS (only really used to make sure we can fix autonomous-generated problems)
    CRServo ballArm;

    @Override
    public void init() {

        //HARDWARE MAP
        leftFront = hardwareMap.dcMotor.get("left front");
        leftBack = hardwareMap.dcMotor.get("left back");
        rightFront = hardwareMap.dcMotor.get("right front");
        rightBack = hardwareMap.dcMotor.get("right back");
        liftMotor = hardwareMap.dcMotor.get("lift motor");
        armMotor = hardwareMap.dcMotor.get("arm motor");

        //SETTING DIRECTIONS
        leftFront.setDirection(LEFTDIRECTION);
        leftBack.setDirection(LEFTDIRECTION);
        rightFront.setDirection(RIGHTDIRECTION);
        rightBack.setDirection(RIGHTDIRECTION);
    }

    @Override
    public void loop() {

        //MOVEMENT FLOATS
        float drive;
        float turn;
        float strafe;

        //CONTROL INVERSION
        float leftTrigger1 = gamepad1.left_trigger;
        if (leftTrigger1 <= 0.4) {
            drive = gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            strafe = gamepad1.left_stick_x;
        } else {
            drive = -gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            strafe = -gamepad1.left_stick_x;
        }

        //DRIVING
        double lfDrive = Range.clip(drive + turn - strafe, -1.0, 1.0);
        double lbDrive = Range.clip(drive + turn + strafe, -1.0, 1.0);
        double rfDrive = Range.clip(drive - turn + strafe, -1.0, 1.0);
        double rbDrive = Range.clip(drive - turn - strafe, -1.0, 1.0);

        //NORMAL DRIVING
        float power = gamepad1.right_trigger;
        leftFront.setPower(dir(lfDrive)*power);
        leftBack.setPower(dir(lbDrive)*power);
        rightFront.setPower(dir(rfDrive)*power);
        rightBack.setPower(dir(rbDrive)*power);

        //ARM MOVEMENT
        float dbSpeed = gamepad2.left_stick_y / 2;
        float dbSpeedSlow = gamepad2.left_stick_y / 4;

        if (gamepad2.right_trigger >= 0.3) {
            armMotor.setPower(dbSpeed);
        }
        armMotor.setPower(dbSpeedSlow);

        // TELEMETRY
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", rfDrive, rbDrive, lbDrive, rbDrive);
        telemetry.update();
    }
    //FINDING DIRECTION METHOD
    public static double dir(double motordir){
        return (motordir/Math.abs(motordir));
    }
}
