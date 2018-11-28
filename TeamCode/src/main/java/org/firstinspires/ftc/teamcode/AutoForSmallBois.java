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

    //MOTORS
    DcMotor leftFront, leftBack, rightFront, rightBack;
    DcMotor armMotor;
    DcMotor liftMotor;

    //SERVOS
    CRServo padLock;
    Servo markerMech;

    //SENSORS
    ModernRoboticsI2cGyro MRGyro;
    IntegratingGyroscope gyro;
    ColorSensor depotSensor;
    DistanceSensor rangeSensorBottom, rangeSensorFront;

    //GYRO SENSOR CODES
    int balanceMove = 250;
    int targetHeadingGlyph = 180;
    int targetHeading = 270;

    //ENCODERS
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
        gyro = (IntegratingGyroscope) MRGyro;
        depotSensor = hardwareMap.get(ColorSensor.class, "depot sensor");
        rangeSensorBottom = hardwareMap.get(DistanceSensor.class, "range sensor bottom");
        rangeSensorFront = hardwareMap.get(DistanceSensor.class, "range sensor front");

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
        turn(leftFront, leftBack, rightFront, rightBack, true, 0.3);
        if (heading > targetHeadingGlyph - 10 && heading < targetHeadingGlyph + 10) {
            move(leftFront, leftBack, rightFront, rightBack,true,0);
            sleep(5000);
        }
    }

        public void landing () {
            double distanceFromGround = rangeSensorBottom.getDistance(DistanceUnit.CM);
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
                move(leftFront, rightFront, leftBack, rightBack, true, .5);
            }
            depositing();
        }
        public void depositRed () {
            while (!(depotSensor.red() > depotSensor.green()) && !(depotSensor.red() > depotSensor.blue())) {
                move(leftFront, rightFront, leftBack, rightBack, true, .5);
            }
            depositing();
        }

        public void depositing () {
            //TODO: test servo, preferably w/ test code
            markerMech.setPosition(0.5); //value for testing
        }



//---------------------------//
//BASIC MOVEMENT METHODS

        public void move (DcMotor motorlf, DcMotor motorrf, DcMotor motorlb, DcMotor motorrb,
        boolean direction, double speed){
            //ONE METHOD TO MOVE FORWARDS AND BACKWARDS
            //values for testing
            if (!direction) { //hopefully makes robo go backwards
                motorlf.setPower(-speed);
                motorrf.setPower(-speed);
                motorlb.setPower(speed);
                motorrb.setPower(speed);
            } else if (direction) {
                motorlf.setPower(speed);
                motorrf.setPower(speed);
                motorlb.setPower(-speed);
                motorrb.setPower(-speed);
            }
        }

        public void strafe (DcMotor motorlf, DcMotor motorrf, DcMotor motorlb, DcMotor motorrb,
        boolean direction, double speed){
            //ONE METHOD TO MOVE FORWARDS AND BACKWARDS
            //values for testing
            if (!direction) { //hopefully makes robo go to the right
                motorlf.setPower(speed);
                motorrf.setPower(-speed);
                motorlb.setPower(speed);
                motorrb.setPower(-speed);
            } else if (direction) {
                motorlf.setPower(-speed);
                motorrf.setPower(speed);
                motorlb.setPower(-speed);
                motorrb.setPower(speed);
            }
        }

        public void turn (DcMotor motorlf, DcMotor motorrf, DcMotor motorlb, DcMotor motorrb,
        boolean direction, double speed){
            //ONE METHOD TO MOVE FORWARDS AND BACKWARDS
            //values for testing
            if (!direction) { //hopefully makes robo turn to the right
                motorlf.setPower(-speed);
                motorrf.setPower(-speed);
                motorlb.setPower(-speed);
                motorrb.setPower(-speed);
            } else if (direction) {
                motorlf.setPower(speed);
                motorrf.setPower(speed);
                motorlb.setPower(speed);
                motorrb.setPower(speed);
            }
        }
}