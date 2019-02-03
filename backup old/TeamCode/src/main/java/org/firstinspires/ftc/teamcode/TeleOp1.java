package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="TeleOp1.0", group="Tele Op")
public class TeleOp1 extends OpMode {

    private ElapsedTime runtime = new ElapsedTime();

    //WHEELS
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    //INTAKE
    Servo leftClamp;
    Servo rightClamp;
    DcMotor armMotor;
    //LIFTING THE ROBOT
    DcMotor liftMotor;

    //DIRECTIONS
    private DcMotor.Direction LEFTDIRECTION = DcMotor.Direction.REVERSE;
    private DcMotor.Direction RIGHTDIRECTION = DcMotor.Direction.FORWARD;

    @Override
    public void init() {
        //HARDWARE MAP FOR WHEELS
        leftFront = hardwareMap.dcMotor.get("left front");
        leftBack = hardwareMap.dcMotor.get("left back");
        rightFront = hardwareMap.dcMotor.get("right front");
        rightBack = hardwareMap.dcMotor.get("right back");
/*        //HARDWARE MAP FOR INTAKE
        leftClamp = hardwareMap.servo.get("left clamp");
        rightClamp = hardwareMap.servo.get("right clamp"); */
        armMotor = hardwareMap.dcMotor.get("arm motor");
        //HARDWARE MAP FOR LIFTING
  //      liftMotor = hardwareMap.dcMotor.get("lift motor ");

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
        float rightTrigger1 = gamepad1.right_trigger;
        if (rightTrigger1 <= 0.4) {
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

        //SLOW DRIVING
        if(gamepad1.left_trigger >= 0.3) {
            leftFront.setPower(lfDrive/3);
            leftBack.setPower(lbDrive/3);
            rightFront.setPower(rfDrive/3);
            rightBack.setPower(rbDrive/3);

        }

        //NORMAL DRIVING
        else{
            leftFront.setPower(lfDrive);
            leftBack.setPower(lbDrive);
            rightFront.setPower(rfDrive);
            rightBack.setPower(rbDrive);
        }

        //TODO: code intake when able to
/*
        //INTAKE
        boolean gamepad2A = gamepad2.a;
        boolean gamepad2B = gamepad2.b;
        boolean gamepad2Y = gamepad2.y;
        if (gamepad2B) { //square clamping
          //  leftClamp.setPosition(0.5); //these no. for testing
            rightClamp.setPosition(0.5); //these no. for testing
        }
        else if (gamepad2A) { //circle clamping
          //  leftClamp.setPosition(0.55); //these no. for testing
            rightClamp.setPosition(0.55); //these no. for testing
        }
        else if (gamepad2Y) { //opening the clamp
           // leftClamp.setPosition(1.0); //these no. for testing
            rightClamp.setPosition(0.0); //these no. for testing
        }
        //MANUAL CLAMPING
       // leftClamp.setPosition(-gamepad2.left_stick_x);
        rightClamp.setPosition(gamepad2.left_stick_x);

*/
        //FLIPPING THE INTAKE UPWARDs
        float armFlip = gamepad2.left_stick_y;
        float rightTrigger2 = gamepad2.right_trigger;
        if (rightTrigger2 >= 0.4) {
            //MORE CONTROLLED, SLOWER FLIPPING
            armMotor.setPower(armFlip/3);
        }
        else {
            //NORMAL FLIPPING
            armMotor.setPower(armFlip);
        }

   /*     //LIFT
        float liftM = gamepad2.right_stick_y;
        liftMotor.setPower(liftM);
*/
        //TELEMETRY
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", rfDrive, rbDrive, lbDrive, rbDrive);
        telemetry.update();
    }
}

