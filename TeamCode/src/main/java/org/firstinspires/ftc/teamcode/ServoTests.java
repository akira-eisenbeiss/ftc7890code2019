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

@TeleOp(name="servo test", group="Tele Op")
public class ServoTests extends OpMode {
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
    Servo sensorSwitch;
    Servo markerMech;

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

        sensorSwitch = hardwareMap.servo.get("sensor switch");
        markerMech = hardwareMap.servo.get("marker mech");



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
 * more efficient to assign one action (unfolding the arm motors) to the joysticks and the other
 * ((de)activating the intake/outake) to the triggers so that the driver doesn't have to
 * reposition their hands.
 */
    sensorSwitch.setPosition(gamepad1.left_stick_y);
    markerMech.setPosition(gamepad1.right_stick_y);



    }
}
