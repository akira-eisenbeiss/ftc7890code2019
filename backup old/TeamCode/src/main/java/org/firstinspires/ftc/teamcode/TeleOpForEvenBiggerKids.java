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
NOTE: a test code, trying to solve some problems with our crservo
 */

@TeleOp(name="TeleOp For Even Bigger Kids", group="Tele Op")
public class TeleOpForEvenBiggerKids extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //MOTORS
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;
    DcMotor liftMotor;
    //DcMotor armMotor;

    //DIRECTIONS
    private DcMotor.Direction LEFTDIRECTION = DcMotor.Direction.REVERSE;
    private DcMotor.Direction RIGHTDIRECTION = DcMotor.Direction.FORWARD;

    //SERVOS (only really used to make sure we can fix autonomous-generated problems)
    CRServo padLock;

    float backward;

    @Override
    public void init() {

        //TODO; update configuration
        //HARDWARE MAP
        leftFront = hardwareMap.dcMotor.get("left front");
        leftBack = hardwareMap.dcMotor.get("left back");
        rightFront = hardwareMap.dcMotor.get("right front");
        rightBack = hardwareMap.dcMotor.get("right back");
        liftMotor = hardwareMap.dcMotor.get("lift motor");
        //  armMotor = hardwareMap.dcMotor.get("arm motor");
        padLock = hardwareMap.crservo.get("padlock");

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
/*
        //CONTROL INVERSION
        float leftTrigger1 = gamepad1.left_trigger;
        if (leftTrigger1 <= 0.4) {
            drive = gamepad1.left_stick_y;
            turn = gamepad1.right_stick_x;
            strafe = gamepad1.left_stick_x;
        } else { */
        drive = -gamepad1.left_stick_y;
        turn = gamepad1.right_stick_x;
        strafe = -gamepad1.left_stick_x;
        //    }
        //we do control inversion in dir

        //DRIVING
        double lfDrive = Range.clip(drive + turn - strafe, -1.0, 1.0);
        double lbDrive = Range.clip(drive + turn + strafe, -1.0, 1.0);
        double rfDrive = Range.clip(drive - turn + strafe, -1.0, 1.0);
        double rbDrive = Range.clip(drive - turn - strafe, -1.0, 1.0);

        //BACKWARDS DRIVING
        float backwards = gamepad1.left_trigger;

        //NORMAL DRIVING
        float power = gamepad1.right_trigger;
        leftFront.setPower(dir(lfDrive,backwards)*power);
        leftBack.setPower(dir(lbDrive,backwards)*power);
        rightFront.setPower(dir(rfDrive,backwards)*power);
        rightBack.setPower(dir(rbDrive,backwards)*power);

      /*  //ARM MOVEMENT
        float dbSpeed = -gamepad2.left_stick_y / 1;
        float dbSpeedSlow = -gamepad2.left_stick_y / 2;

        if (gamepad2.right_trigger >= 0.3) {
            armMotor.setPower(dbSpeedSlow);
        }
        armMotor.setPower(dbSpeed);
*/
        //LIFTING
        float liftpower = gamepad2.right_stick_y;
        //       liftMotor.setPower(liftpower / 4); //powered down for testing
        float down = gamepad2.left_trigger;
        liftMotor.setPower(liftDir(liftpower) * down);

        //HOOKING WITH PADLOCK
        boolean gamepad2A = gamepad2.a;
        boolean gamepad2B = gamepad2.b;
        boolean gamepad2X = gamepad2.x;

        int servo = 0;
        //ASSIGN TO BUTTONS
        if (gamepad2B) { //close padlock
            servo = 1;
        }
        else if (gamepad2A) { //open padlock
            servo = 2;
        }
        else if (gamepad2X){
            servo = 0;
        }

        //CODE FOR PRESSING BUTTONS
        if(servo == 0){
            padLock.setPower(0);
        }
        else if (servo == 2){
            padLock.setPower(-1);
        }
        else if (servo == 1){
            padLock.setPower(1);
        }

        // TELEMETRY
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", rfDrive, rbDrive, lbDrive, rbDrive);
        telemetry.update();
    }
    //FINDING DIRECTION METHOD
    public static double dir(double motordir, double backwards){
        if(backwards >= 0.15){
            return ((motordir/Math.abs(motordir))*-1);
        }
        return (motordir/Math.abs(motordir));
    }

    // BASICALLY FINDS WHETHER DRIVER IS MAKING THE MOTOR GO UP OR DOWN
    //RETURNS EITHER A 1 OR -1
    public static double liftDir(double liftdir) {
        return (liftdir/Math.abs(liftdir));
    }


}
