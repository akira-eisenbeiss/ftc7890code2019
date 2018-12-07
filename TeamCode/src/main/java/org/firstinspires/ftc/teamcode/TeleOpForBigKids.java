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
    DcMotor armMotor;

    //DIRECTIONS
    private DcMotor.Direction LEFTDIRECTION = DcMotor.Direction.REVERSE;
    private DcMotor.Direction RIGHTDIRECTION = DcMotor.Direction.FORWARD;

    //SERVOS (only really used to make sure we can fix autonomous-generated problems)
    CRServo padLock;
    public static int servoCntr;

    float backward;
    int armValue = 0;

    @Override
    public void init() {

        //TODO; update configuration
        //HARDWARE MAP
        leftFront = hardwareMap.dcMotor.get("left front");
        leftBack = hardwareMap.dcMotor.get("left back");
        rightFront = hardwareMap.dcMotor.get("right front");
        rightBack = hardwareMap.dcMotor.get("right back");
        liftMotor = hardwareMap.dcMotor.get("lift motor");
        armMotor = hardwareMap.dcMotor.get("arm motor");
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
        leftFront.setPower(dir(lfDrive, backwards) * power);
        leftBack.setPower(dir(lbDrive, backwards) * power);
        rightFront.setPower(dir(rfDrive, backwards) * power);
        rightBack.setPower(dir(rbDrive, backwards) * power);

        //ARM MOVEMENT
        float armSpeed = -gamepad2.left_trigger;
        float armControl = gamepad2.left_stick_y;
        armMotor.setPower(liftDir(armControl) * armSpeed);

        boolean gamepad2X = gamepad2.x;
        if (gamepad2X) {
            if(armValue == 1){
                armValue ^= 1;
                telemetry.addData("ARM", "FULL");
                armSpeed = armSpeed*2;
                telemetry.update();
            }
            else if(armValue == 0){
                armSpeed = armSpeed/2;
                armValue ^= 0;
                telemetry.addData("ARM", "HALF");
                telemetry.update();
            }
        }

    //LIFTING
    float liftpower = gamepad2.right_stick_y;
    //       liftMotor.setPower(liftpower / 4); //powered down for testing
    float liftControl = gamepad2.right_trigger;
    liftMotor.setPower(liftDir(liftpower) *liftControl);


    //HOOKING WITH PADLOCK
    boolean gamepad1B = gamepad1.b;
    boolean gamepad1Y = gamepad1.y;
    boolean gamepad1X = gamepad1.x;

        //ASSIGN TO BUTTONS
        if (gamepad1Y) { //close padlock
            servoCntr = 1;
        }
        else if (gamepad1B) { //open padlock
            servoCntr = 2;
        }
        else if (gamepad1X) {
            servoCntr = 0;
        }

        //CODE FOR PRESSING BUTTONS
        if(servoCntr == 0){
            padLock.setPower(0.0);
        }
        else if (servoCntr == 1){
            padLock.setPower(0.5);
        }
        else if (servoCntr == 2) {
            padLock.setPower(-0.5);
        }

    // TELEMETRY
        telemetry.addData("Status","Run Time: "+runtime.toString());
        telemetry.addData("Motors","left (%.2f), right (%.2f)",rfDrive,rbDrive,lbDrive,rbDrive);
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
