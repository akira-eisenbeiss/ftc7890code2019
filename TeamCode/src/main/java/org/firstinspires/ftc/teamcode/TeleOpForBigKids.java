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
author: 7890 Software (akira-eisenbeiss, ErinZ)
GOALS: 2019, deposite silver minerals, possibly also gold, lower and raise on the lander
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
    DcMotor armMotor1;
    DcMotor armMotor2;
    DcMotor intake;

    //DIRECTIONS
    private DcMotor.Direction LEFTDIRECTION = DcMotor.Direction.REVERSE;
    private DcMotor.Direction RIGHTDIRECTION = DcMotor.Direction.FORWARD;

    int armValue = 0;
    int intakeCntr = 0;

    @Override
    public void init() {

        //TODO; update configuration
        //HARDWARE MAP
        leftFront = hardwareMap.dcMotor.get("left front");
        leftBack = hardwareMap.dcMotor.get("left back");
        rightFront = hardwareMap.dcMotor.get("right front");
        rightBack = hardwareMap.dcMotor.get("right back");
        liftMotor = hardwareMap.dcMotor.get("lift motor");
        armMotor1 = hardwareMap.dcMotor.get("arm motor 1");
        armMotor2 = hardwareMap.dcMotor.get("arm motor 2");
        intake = hardwareMap.dcMotor.get("intake motor");

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

        drive = -gamepad1.left_stick_y;
        turn = gamepad1.right_stick_x;
        strafe = -gamepad1.left_stick_x;

        //DRIVING
        double lfDrive = Range.clip(drive + turn - strafe, -1.0, 1.0);
        double lbDrive = Range.clip(drive + turn + strafe, -1.0, 1.0);
        double rfDrive = Range.clip(drive - turn + strafe, -1.0, 1.0);
        double rbDrive = Range.clip(drive - turn - strafe, -1.0, 1.0);

        //NORMAL DRIVING
        leftFront.setPower(lfDrive);
        leftBack.setPower(lbDrive);
        rightFront.setPower(rfDrive);
        rightBack.setPower(rbDrive);

        //ARM MOVEMENT
        float armControl = gamepad2.right_stick_y;
        float armControl2 = gamepad2.left_stick_y;
        armMotor1.setPower(armControl);
        armMotor2.setPower(armControl2);

        //INTAKE
        boolean gamepad1B = gamepad1.b;
        boolean gamepad1Y = gamepad1.y;
        boolean gamepad1X = gamepad1.x;

        //ASSIGN TO BUTTONS
        if (gamepad1Y) {
            intakeCntr = 1;
        } else if (gamepad1B) {
            intakeCntr = 2;
        } else if (gamepad1X) {
            intakeCntr = 0;
        }

        //CODE FOR PRESSING BUTTONS
        if (intakeCntr == 0) {
            intake.setPower(0.0);
        } else if (intakeCntr == 1) {
            intake.setPower(1);
        } else if (intakeCntr == 2) {
            intake.setPower(-1);
        }


        //LIFTING
        float liftControlUp = gamepad1.right_trigger;
        float liftControlDown = gamepad1.left_trigger;
        liftMotor.setPower(liftControlUp);
        liftMotor.setPower(-liftControlDown);

        // TELEMETRY
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", rfDrive, rbDrive, lbDrive, rbDrive);
        telemetry.update();
    }
}
