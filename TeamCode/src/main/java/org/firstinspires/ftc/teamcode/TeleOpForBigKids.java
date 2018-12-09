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
GOALS: 2019, deposit silver minerals, possibly also gold, lower and raise on the lander
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
    //sets directions to the wheels
    private DcMotor.Direction LEFTDIRECTION = DcMotor.Direction.REVERSE;
    private DcMotor.Direction RIGHTDIRECTION = DcMotor.Direction.FORWARD;

    //SERVOS
    //this servo is used to latch us onto the lander
    CRServo padLock;

    //USED TO TOGGLE BETWEEN DIFFERENT
    //MODES WHICH CHANGE THE DIRECTION OF
    //OUR SERVO, ex: Padlock in, Padlock out
    public static int servoCntr;

    //USED IN XOR GATE [see later lines]
    int armValue = 0;

    @Override
    public void init() {

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

        drive = -gamepad1.left_stick_y;
        turn = gamepad1.right_stick_x;
        strafe = -gamepad1.left_stick_x;

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
        armMotor.setPower(-liftDir(armControl) * armSpeed);


        //USES AN XOR GATE IN ORDER TO TOGGLE BETWEEN BUTTONS
        //With 0 and 1 or 1 and 0 we get --> 1
        //With 0 and 0 or 1 and 1 we get --> 0
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
    /*
     //FINDING DIRECTION METHOD
     divides the value of our joystick by the absolute value
     of itself to get either a 1 or -1, this gives us the direction
     that the motor should turn in ex: forward/backward, left/right
     USING THIS METHOD, drivers will be able to separate speed and
     direction when driving the robot.
    */
    public static double dir(double motordir, double backwards){
        if(backwards >= 0.15){
            return ((motordir/Math.abs(motordir))*-1);
        }
        return (motordir/Math.abs(motordir));
    }

    // BASICALLY FINDS WHETHER DRIVER IS MAKING THE MOTOR GO UP OR DOWN
    // RETURNS EITHER A 1 OR -1
    public static double liftDir(double liftdir) {
            return (liftdir/Math.abs(liftdir));
    }
}
