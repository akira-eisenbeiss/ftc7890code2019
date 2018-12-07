package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.Locale;
import org.firstinspires.ftc.robotcontroller.external.samples.ConceptVuforiaNavigation;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuMarkInstanceId;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackableDefaultListener;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
/*
  7890 Space Lions 2018 "FULL AUTO FINAL"
  GOALS: lowering robot, deposit marker, parking
*/
@Autonomous(name="autonomous for small", group="LinearOpMode")
public class AutoForSmallBois extends LinearOpMode {
    private ElapsedTime runtime = new ElapsedTime();

    //-----MOTORS and SERVOS-----

    /*
      WHEEL MOTORS:
      These motors control our mechanum wheels. The front
    of the robot is the side with our lifting mech
  */
    DcMotor leftFront, leftBack, rightFront, rightBack;
    /*
      MECHANISM MOTORS:
    These motors control our primary point-scoring
    mechanisms. The intake mechanism and the lifting mech.
  */
    DcMotor armMotor;
    DcMotor liftMotor;
    /*
        MECHANISM SERVOS:
    These servos control smaller tasks, such as unlatching
    the hook on the robot and releasing the marker from the robot.
    */
    CRServo padLock;
    Servo markerMech;

    //-----SENSORS and VARIABLES-----

    //SENSORS:
    //Our range sensor is used to detect our distance to the wall.
    //DistanceSensor rangeSensorBottom, rangeSensorFront;
    ModernRoboticsI2cRangeSensor rangeSensor;
    //The optical distance sensor is used to detect our height off the grou n d.
   // OpticalDistanceSensor odsSensor;
    //Our gyro sensor allows us to detect and make accurate turns.
    ModernRoboticsI2cGyro MRGyro;
    IntegratingGyroscope gyro;
    //Our color sensor is used to detect colored lines on the ground.
    ColorSensor depotSensor;

    DistanceSensor distanceSensor;
    //VARIABLES
    //These variables are used in our gyro sensor code in order to find
    //the correct angle and turn to them.
    int balanceMove = 250;
    int targetHeadingGlyph = 180;
    int targetHeading = 270;
    //ENCODERS
    //These encoders are used to see how much we are moving.
    static final double COUNTS_PER_MOTOR_REV = 1440;    // eg: TETRIX Motor Encoder
    static final double DRIVE_GEAR_REDUCTION = 2.0;     // This is < 1.0 if geared UP
    static final double WHEEL_DIAMETER_INCHES = 4.0;     // For figuring circumference
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double DRIVE_SPEED = 0.5;
    static final double TURN_SPEED = 0.5;

    //VUFORIA
    VuforiaLocalizer vuforia;

    //LANDING VALUE
    double landingValue = 0.5;

    @Override
    public void runOpMode() throws InterruptedException {

        //MOTORS
        leftFront = hardwareMap.dcMotor.get("left front");
        leftBack = hardwareMap.dcMotor.get("left back");
        rightFront = hardwareMap.dcMotor.get("right front");
        rightBack = hardwareMap.dcMotor.get("right back");

        //SENSORS
        MRGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        depotSensor = hardwareMap.get(ColorSensor.class, "depot sensor");
     /*   rangeSensorBottom = hardwareMap.get(DistanceSensor.class, "range sensor bottom");
        rangeSensorFront = hardwareMap.get(DistanceSensor.class, "range sensor front");
*/
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range sensor");
     //   odsSensor = hardwareMap.get(OpticalDistanceSensor.class, "ODS");
        distanceSensor = hardwareMap.get(DistanceSensor.class, "distance sensor");

        //SERVOS
        padLock = hardwareMap.crservo.get("padlock");
        markerMech = hardwareMap.servo.get("marker");
        waitForStart();
          /*
          Below are our methods for
          autonomous...
          */

        landing();
        MRGyro.calibrate();

    }

    public void gyro(int angle) {
        int heading = MRGyro.getHeading();
        move(leftFront, leftBack, rightFront, rightBack, "RIGHT", 0.3);
        if (heading > targetHeadingGlyph - 10 && heading < targetHeadingGlyph + 10) {
            move(leftFront, leftBack, rightFront, rightBack,"FORWARD",0);
            sleep(5000);
        }
    }

    public void landing () {
        double distanceFromGround = distanceSensor.getDistance(DistanceUnit.CM);
        while (distanceFromGround > 6) { //this value is for testing
            liftMotor.setPower(landingValue); //TODO: test landingValue
        }
        if (distanceFromGround <= 6.0) {
            liftMotor.setPower(0);
            padLock.setPower(-0.5); //TODO: test values to see direction of crservo but also how long it takes
            sleep(500);
            liftMotor.setPower(-landingValue); //basically just rewinds the motor so that we can retract the lift
        }
    }

    public void depositBlue () {
        while (!(depotSensor.blue() > depotSensor.green()) && !(depotSensor.blue() > depotSensor.red())) {
            move(leftFront, rightFront, leftBack, rightBack, "FORWARD", .5);
        }
        depositing();
    }
    public void depositRed () {
        while (!(depotSensor.red() > depotSensor.green()) && !(depotSensor.red() > depotSensor.blue())) {
            move(leftFront, rightFront, leftBack, rightBack, "FORWARD", .5);
        }
        depositing();
    }

    public void depositing () {
        //TODO: test servo, preferably w/ test code
        markerMech.setPosition(0.5); //value for testing
    }

    public void crater() {
        double distanceValue = rangeSensor.getDistance(DistanceUnit.INCH);
        while(distanceValue > 6){
            move(leftFront, leftBack, rightFront, rightBack, "FORWARDS", 0.5);
        }
        while(distanceValue <= 6){
        }
    }



//---------------------------//
//BASIC MOVEMENT METHODS

    public void move (DcMotor motorlf, DcMotor motorrf, DcMotor motorlb, DcMotor motorrb,
                      String direction, double speed){
        //ONE METHOD TO MOVE FORWARDS AND BACKWARDS
        //values for testing
        if (direction.equals("BACK")) { //hopefully makes robo go backwards
            motorlf.setPower(-speed);
            motorrf.setPower(-speed);
            motorlb.setPower(speed);
            motorrb.setPower(speed);
        } else if (direction.equals("FORWARDS")) {
            motorlf.setPower(speed);
            motorrf.setPower(speed);
            motorlb.setPower(-speed);
            motorrb.setPower(-speed);
        }
        else if (direction.equals("RIGHT")) { //hopefully makes robo go to the right
            motorlf.setPower(speed);
            motorrf.setPower(-speed);
            motorlb.setPower(speed);
            motorrb.setPower(-speed);
        } else if (direction.equals(("LEFT")){
            motorlf.setPower(-speed);
            motorrf.setPower(speed);
            motorlb.setPower(-speed);
            motorrb.setPower(speed);
        } else if (direction.equals("TURN RIGHT")) { //hopefully makes robo turn to the right i hope so too
            motorlf.setPower(-speed);
            motorrf.setPower(-speed);
            motorlb.setPower(-speed);
            motorrb.setPower(-speed);
        } else if (direction.equals("TURN LEFT")) {
            motorlf.setPower(speed);
            motorrf.setPower(speed);
            motorlb.setPower(speed);
            motorrb.setPower(speed);
        } else if (direction.equals("STOP")) {
        motorlf.setPower(0.0);
        motorrf.setPower(0.0);
        motorlb.setPower(0.0);
        motorrb.setPower(0.0);
        }
    }
}
