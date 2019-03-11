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

@TeleOp(name="NEW TELEOP CHAMPS", group="Tele Op")
public class NewTeleOp extends OpMode {
    private ElapsedTime runtime = new ElapsedTime();

    /*
    ---MOTORS---
    In this section of our code, we declare the various motors on our robot.
    We use these motors to do everything from driving to controlling our
    point scoring mechanisms.
    */

    // The four motors controlling the wheels on our robot:
    // [NeveRest 40(s)]
    DcMotor leftFront;
    DcMotor rightFront;
    DcMotor leftBack;
    DcMotor rightBack;

    // The motor for our lift:
    // [NeveRest 60]
    DcMotor liftMotor;

    DcMotor extendArm;
    DcMotor armMotor;

    DcMotor intakeMotor;

    //DIRECTIONS
    //Controls the directions of our robot's motors.
    private DcMotor.Direction LEFTDIRECTION = DcMotor.Direction.REVERSE;
    private DcMotor.Direction RIGHTDIRECTION = DcMotor.Direction.FORWARD;

    //Servo outtake;

    int intakeCntr;

    @Override
    public void init() {

        /*
        In this section of the code, we hook up the different hardware pieces to their names on the phone.
        */
        leftFront = hardwareMap.dcMotor.get("left front");
        leftBack = hardwareMap.dcMotor.get("left back");
        rightFront = hardwareMap.dcMotor.get("right front");
        rightBack = hardwareMap.dcMotor.get("right back");
        liftMotor = hardwareMap.dcMotor.get("lift motor");

        extendArm = hardwareMap.dcMotor.get("extend arm");
        intakeMotor = hardwareMap.dcMotor.get("intake motor");
        armMotor = hardwareMap.dcMotor.get("arm motor");

        //outtake = hardwareMap.servo.get("outtake");

        /*
        In this section, we are setting directions for the motors on the robot.
        This prevents issues in the wheels. As all the wheels would naturally
        spin the same way, when driving our robot would rotate instead of moving forward.
        This means we need to make sure that forward is the same for both the left and
        right sides of our robot.
        */
        leftFront.setDirection(LEFTDIRECTION);
        leftBack.setDirection(LEFTDIRECTION);
        rightFront.setDirection(RIGHTDIRECTION);
        rightBack.setDirection(RIGHTDIRECTION);

        //outtake.setPosition(1.0);
    }

    @Override
    public void loop() {

/* In this section of the code we create variables for controlling the movement of the robot,
 * the motors on the arm, and the intake. These are then mapped to buttons on our controllers for
 * our drivers (1 & 2). We took into consideration the primary role of each driver as well as the
 * location of the buttons controller and buttons for which an action is assigned. We also thought
 * about how controls are set up in games to increase the ability to multi-task, as well as what
 * types of actions are usually associated with a button/trigger to make driver training easier.
--------------------
 * For example, (as with Driver 2) for actions that can be done simultaenously to save time, it is
 * more efficient to assign one action (unfolding the arm motor) to the joystick and the other
 * ((de)activating the intake/outake) to the buttons so that the driver doesn't have to
 * reposition their hands.
 */
        //In this section, we are creating floats for drive, turn, and strafe.
        float drive;
        float turn;
        float strafe;

        //Setting controls on the gamepad for drive, turn, and strafe
        drive = gamepad1.left_stick_y;
        turn = -gamepad1.right_stick_x;
        strafe = gamepad1.left_stick_x;

        //Setting driver mechanics for the gamepad
        double lfDrive = Range.clip(drive + turn - strafe, -1.0, 1.0);
        double lbDrive = Range.clip(drive + turn + strafe, -1.0, 1.0);
        double rfDrive = Range.clip(drive - turn + strafe, -1.0, 1.0);
        double rbDrive = Range.clip(drive - turn - strafe, -1.0, 1.0);


        //This ratio allows us to slow down our driving to be more precise.
        //This allows our drivers to not be as sensitive while driving, as
        //it can be hard to be calm and accurate during the middle of a match.
        double ratio = (gamepad1.right_trigger);
        //This code sets the power of the various motors to the variable drive
        leftFront.setPower(lfDrive - ratio);
        leftBack.setPower(lbDrive - ratio);
        rightFront.setPower(rfDrive - ratio);
        rightBack.setPower(rbDrive - ratio);

        if(gamepad1.x){
            leftFront.setPower(0.3);
            leftBack.setPower(0.3);
            rightFront.setPower(-0.3);
            rightBack.setPower(-0.3);
        }
        if(gamepad1.b){
            leftFront.setPower(-0.3);
            leftBack.setPower(-0.3);
            rightFront.setPower(0.3);
            rightBack.setPower(0.3);
        }



        // LIFTING
        /*
         * The following code creates variables for the lift controls. We decided to map these to
         * the bumpers for driver 1. This would allow the driver to control the speed of the lift
         * motors for more accuracy instead of a constant speed which would likely force us to
         * guess the timing. We no longer need to make multiple unnessary adjustments to get the
         * lift where we need it.
         */
        if(gamepad1.right_bumper){
            liftMotor.setPower(1);
        }
        else if(gamepad1.left_bumper){
            liftMotor.setPower(-1);
        }
        else{
            liftMotor.setPower(0);
        }

        //Arm
        /*
         * Because we use joysticks to extend the arm in and out as well as to flip the whole mechanism,
         * all we need to do is just take the values that the joysticks are outputting and plug them in as input
         * values to power each of the motors.
         */
        armMotor.setPower(gamepad2.left_stick_y);
        extendArm.setPower(-gamepad2.right_stick_y);

        //We use a switch-case here in order to use the buttons in order to switch
        //between the intake spinning in, out, or being off.
        if (gamepad2.y){
            intakeCntr = 0;
        }
        else if (gamepad2.a) {
            intakeCntr = 1;
        }
        else if (gamepad2.x) {
            intakeCntr = 2;
        }
        switch (intakeCntr) {
            case 0:
                intakeMotor.setPower(0.0);
                break;
            case 1:
                intakeMotor.setPower(1.0);
                break;
            case 2:
                intakeMotor.setPower(-1.0);
                break;
        }

        // TELEMETRY
        /* The telemetry displays useful information on the phone such as
         * How long the code has been running for and the power sent to our motors.
         */
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)",
                rfDrive, rbDrive, lbDrive, rbDrive);
        telemetry.update();
    }
}
