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
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import com.disnodeteam.dogecv.CameraViewDisplay;
import com.disnodeteam.dogecv.DogeCV;
import com.disnodeteam.dogecv.detectors.roverrukus.GoldAlignDetector;
import com.disnodeteam.dogecv.detectors.roverrukus.SamplingOrderDetector;
/*
7890 Space Lions 2019 "Crater Autonomous for Big Kids"
author: 7890 Software (Akira, Erin, Stephen, Kyra, Anthony)
GOALS: 2019, land, sample, deposit team marker, park in crater
 */

@Autonomous(name="Landing ONLY", group="LinearOpMode")
public class OnlyDrop extends LinearOpMode {

    /*
     * MOTORS, SERVOS, and SENSORS
     * In this section of the code, we declare our motors, servos, and sensors
     */

    //This code declares the four wheels on our robot:
    DcMotor leftFront, leftBack, rightFront, rightBack;

    //The motor for our lift:
    DcMotor liftMotor;

    //Our range sensor that uses ODS and ultrasonic to detect our distance from objects:
    //Used to detect the wall

    //Our gyro sensor that calibrates at a target heading and detects our angle away from that heading
    //We can use this to accurately turn
    //ModernRoboticsI2cGyro MRGyro;

    //Our range sensor that uses ODS and ultrasonic to detect our distance from objects:
    //Used to detect the distance from the ground
    ModernRoboticsI2cRangeSensor doubleSensor;

    Servo sensorSwitch;
    Servo markerMech;
    //ModernRoboticsI2cRangeSensor sideSensor1, sideSensor2;

    boolean detected = false;
    GoldAlignDetector detector;

    @Override
    public void runOpMode() throws InterruptedException {

        detector = new GoldAlignDetector();
        detector.init(hardwareMap.appContext, CameraViewDisplay.getInstance(), 0, false);
        detector.useDefaults();

        detector.alignSize = 900;
        detector.alignPosOffset = 0;
        detector.downscale = 0.4;

        detector.areaScoringMethod = DogeCV.AreaScoringMethod.MAX_AREA;
        detector.maxAreaScorer.weight = 0.005;

        detector.ratioScorer.weight = 5;
        detector.ratioScorer.perfectRatio = 1.0;

        detector.enable();

        /*
         * The HARDWARE MAP
         * Here we hook up the hardware pieces (Motors, Servos, Sensors) to their names on the phone.
         */
        //MOTORS
        //These motors are hooked up to their respective names that we assigned them during hardware mapping on the phone.
        leftFront = hardwareMap.dcMotor.get("left front");
        leftBack = hardwareMap.dcMotor.get("left back");
        rightFront = hardwareMap.dcMotor.get("right front");
        rightBack = hardwareMap.dcMotor.get("right back");
        liftMotor = hardwareMap.dcMotor.get("lift motor");

        //SERVOS
        sensorSwitch = hardwareMap.servo.get("sensor switch");
        markerMech = hardwareMap.servo.get("marker mech");


        //SENSORS
       // MRGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        doubleSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "depot sensor");

        //MRGyro.calibrate();
        sleep(4000);

        /*
         * AUTONOMOUS MAIN METHOD
         * The start of our autonomous code
         */
        markerMech.setPosition(-1);
        sensorSwitch.setPosition(0);
        waitForStart();

        landing();
        sensorSwitch.setPosition(-0.5);
        sampling();

        //deposit();
        //crater();
    }

    /*
     * LANDING Method
     * This method allows us to land our robot by detecting our distance
     * from the ground using our MR range sensor. Once we reach the ground
     * we rotate our robot in order to unhook.
     */
    public void landing() {
        //cases, naming, data types
        //double distanceFromGround = depotSensor.getDistance(DistanceUnit.INCH);

        boolean landed = false;
        while (!landed) {
            if (doubleSensor.getDistance(DistanceUnit.INCH) <= 1.6) {
                liftMotor.setPower(0);
                //GETS US OFF THE HOOK
                liftMotor.setPower(0.8);
                sleep(2000);
                liftMotor.setPower(0.0);
                move("east", 0.8);
                sleep(500);
                stopMove();
                liftMotor.setPower(-1.0);
                sleep(1400);
                liftMotor.setPower(0.0);
                move("west", 0.8);
                sleep(250);
                telemetry.addLine("moved off hook");
                telemetry.update();
                stopMove();
                landed = true;
            } else if (doubleSensor.getDistance(DistanceUnit.INCH) > 1.6) {
                liftMotor.setPower(1.0);
                telemetry.addData("distance", doubleSensor.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }
        }
    }

    public void sampling() {
        move("south", 0.3);
        sleep(500);
        stopMove();

        if (detect() && !detected) {
            telemetry.addData("pos", "center");
            telemetry.update();
            detected = true;
        } else if (!detect() && !detected) {
            //gyro(45, 'L');
            telemetry.addData("pos", "NOT center");
            telemetry.addLine("turning left");
            telemetry.update();
            if (detect() && !detected) {
                telemetry.addData("pos", "left");
                telemetry.update();
                detected = true;
            } else if (!detect() && !detected) {
                //gyro(315, 'R');
                telemetry.addData("pos", "NOT left");
                telemetry.addLine("turning right");
                telemetry.update();
                if (detect() && !detected) {
                    telemetry.addData("pos", "right");
                    telemetry.update();
                    detected = true;
                } else if (!detect() && !detected) {
                    telemetry.addData("pos", "NOT right, giving up");
                    telemetry.update();
                }
            }
        }
        if (detected) {
            telemetry.addLine("we are now moving");
            telemetry.update();
            sleep(4000);
        }
    }

    public boolean detect() {
        if (detector.getAligned()) {
            return true;
        } else if (!detector.getAligned()) {
            return false;
        } else {
            telemetry.addLine("Oops, an error did a happen.");
            telemetry.update();
            return false;
        }
    }

    /*
     * CRATER Method
     * In our crater method we turn our robot
     * so that it is facing the crater and then
     * drive our robot to park.
     */

    /*
     * MOVEMENT Method
     * In this code we use a switch case in order to create
     * more code-efficient robot driving. This means that instead
     * of having to individually control each motor everytime we
     * want to move, we can instead just call the move() method.
     * And specify the wanted direction with a case-valid string.
     * This makes our program a lot shorter than it would
     * normally have been, and makes it easier to program and read our code. It
     * assigns the proper motor speed assigned when the method is called,
     * and sets the rotation of the motors so that the robot moves in the specified direction
     *(with either positive or negative speed)
     */
    public void move(String direction, double speed) {
        switch (direction) {
            case "north":
                //robot moves backwards
                leftFront.setPower(speed);
                rightFront.setPower(-speed);
                leftBack.setPower(speed);
                rightBack.setPower(-speed);
                break;
            case "south":
                //robot moves forwards
                leftFront.setPower(-speed);
                rightFront.setPower(speed);
                leftBack.setPower(-speed);
                rightBack.setPower(speed);
                break;
            case "east":
                //robot strafes right
                leftFront.setPower(-speed);
                rightFront.setPower(-speed);
                leftBack.setPower(speed);
                rightBack.setPower(speed);
                break;
            case "west":
                //robot strafes left
                leftFront.setPower(speed);
                rightFront.setPower(speed);
                leftBack.setPower(-speed);
                rightBack.setPower(-speed);
                break;
            case "cc":
                //robot turns clockwise(to the right)
                leftFront.setPower(-speed);
                rightFront.setPower(-speed);
                leftBack.setPower(-speed);
                rightBack.setPower(-speed);
                break;
            case "ccw":
                //robot turns counterclockwise(to the left)
                leftFront.setPower(speed);
                rightFront.setPower(speed);
                leftBack.setPower(speed);
                rightBack.setPower(speed);
                break;
            case "north east":
                leftFront.setPower(speed);
                rightFront.setPower(0.0);
                leftBack.setPower(0.0);
                rightBack.setPower(-speed);
                break;
            case "north west":
                leftFront.setPower(0.0);
                rightFront.setPower(-speed);
                leftBack.setPower(speed);
                rightBack.setPower(0.0);
                break;
            case "south east":
                leftFront.setPower(0.0);
                rightFront.setPower(speed);
                leftBack.setPower(-speed);
                rightBack.setPower(0.0);
                break;
            case "south west":
                leftFront.setPower(-speed);
                rightFront.setPower(0.0);
                leftBack.setPower(0.0);
                rightBack.setPower(speed);
                break;
        }
    }
    /*
     * STOP Method
     * Similar to the move method, our stop method
     * takes in the different wheels as parameters
     * and sets all of their powers to 0, stopping
     * them. This reduces the number of lines needed
     * to stop the robot from four to only one.
     */
    public void stopMove() {
        //robot stops moving
        leftFront.setPower(0.0);
        rightBack.setPower(0.0);
        leftBack.setPower(0.0);
        rightFront.setPower(0.0);
    }
}
