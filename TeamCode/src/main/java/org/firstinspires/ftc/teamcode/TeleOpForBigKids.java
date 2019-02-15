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

@TeleOp(name="TeleOp For Big Kids", group="Tele Op")
public class TeleOpForBigKids extends OpMode {
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
    
    // The two motors on our double jointed robot arm.
    // Arm Motor 1 is the motor directly attached to the robot
    // [NeveRest 60, Core Hex Motor (respectively)]
    DcMotor armMotor1;
    DcMotor armMotor2;

    // The motor for our intake
    // [Core Hex Motor]
    DcMotor intake;

    Servo lock;

    //DIRECTIONS
    //Controls the directions of our robot's motors.
    private DcMotor.Direction LEFTDIRECTION = DcMotor.Direction.REVERSE;
    private DcMotor.Direction RIGHTDIRECTION = DcMotor.Direction.FORWARD;

    int armValue = 0;
    int intakeCntr = 0;

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
        armMotor1 = hardwareMap.dcMotor.get("arm motor 1");
        armMotor2 = hardwareMap.dcMotor.get("arm motor 2");
        intake = hardwareMap.dcMotor.get("intake motor");

        lock = hardwareMap.servo.get("lock");

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

        lock.setPosition(1.0);
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
        //In this section, we are creating floats for drive, turn, and strafe. 
        float drive;
        float turn;
        float strafe;
        
        //Setting controls on the gamepad for drive, turn, and strafe
        drive = -gamepad1.left_stick_y;
        turn = gamepad1.right_stick_x;
        strafe = -gamepad1.left_stick_x;

        //Setting driver mechanics for the gamepad
        double lfDrive = Range.clip(drive + turn - strafe, -1.0, 1.0);
        double lbDrive = Range.clip(drive + turn + strafe, -1.0, 1.0);
        double rfDrive = Range.clip(drive - turn + strafe, -1.0, 1.0);
        double rbDrive = Range.clip(drive - turn - strafe, -1.0, 1.0);

        //This code sets the power of the various motors to the variable drive
        leftFront.setPower(lfDrive);
        leftBack.setPower(lbDrive);
        rightFront.setPower(rfDrive);
        rightBack.setPower(rbDrive);

        //This code controls the movement of the arm
        float armControl = gamepad2.left_stick_y;
        float armControl2 = gamepad2.right_stick_y;
        armMotor1.setPower(-armControl);
        armMotor2.setPower(armControl2);

        //Intake buttons
        boolean gamepad2B = gamepad2.b;
        boolean gamepad2Y = gamepad2.y;
        boolean gamepad2X = gamepad2.x;

        //These are the various buttons assigned to the intake mechanism, and are mapped on Driver 2's controller. Y toggles the intake. B toggles the outtake. And X stops the motor completely.
        if (gamepad2Y) {
            intakeCntr = 1;
        } else if (gamepad2B) {
            intakeCntr = 2;
        } else if (gamepad2X) {
            intakeCntr = 0;
        }

        //This code checks if the intake control is 0, -1 or 1. The associated buttons above which control this toggle the off (0), turn the motor on in order to intake (1), or reverse it to outtake (-1)
        if (intakeCntr == 0) {
            intake.setPower(0.0);
        } else if (intakeCntr == 1) {
          //Intakes
            intake.setPower(1);
        } else if (intakeCntr == 2) {
          //Outtakes
            intake.setPower(-1);
        }

        boolean gamepad1b = gamepad1.b;
        boolean gamepad1a = gamepad1.a;
        boolean gamepad1x = gamepad1.x;
        boolean gamepad1y = gamepad1.y;
        if (gamepad1a){
            lock.setPosition(1);
        }else if(gamepad1b){
            lock.setPosition(-1);
        }
        else if(gamepad1x){
            lock.setPosition(2.0);
        }
        else if(gamepad1y){
            lock.setPosition(0);
        }

        //LIFTING
        /*
        * The following code creates variables for the lift controls. We decided to map these to 
        * the triggers for Driver 1. This would allow the driver to control the speed of the lift 
        * motors for more accuracy instead of a constant speed which would likely force us to 
        *guess the timing. We no longer need to make multiple unnessary adjustments to get the lift where we need it.
        */
        float liftControlUp = gamepad1.right_trigger;
        float liftControlDown = gamepad1.left_trigger;
        liftMotor.setPower(liftControlUp);
        liftMotor.setPower(-liftControlDown);

        // TELEMETRY
        /* The telemetry displays useful information on the phone such as
         * How long the code has been running for and the power sent to our motors.
         */
        telemetry.addData("Status", "Run Time: " + runtime.toString());
        telemetry.addData("Motors", "left (%.2f), right (%.2f)", rfDrive, rbDrive, lbDrive, rbDrive);
        telemetry.update();
    }
}
