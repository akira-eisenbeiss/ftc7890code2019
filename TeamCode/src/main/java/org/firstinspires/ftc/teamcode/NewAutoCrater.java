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

@Autonomous(name="NEW AUTO CRATER 2.0", group="LinearOpMode")
public class NewAutoCrater extends LinearOpMode {

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
    ModernRoboticsI2cGyro MRGyro;

    //Our range sensor that uses ODS and ultrasonic to detect our distance from objects:
    //Used to detect the distance from the ground
    ModernRoboticsI2cRangeSensor doubleSensor;
    ModernRoboticsI2cRangeSensor forwardSensor;

    //Servo sensorSwitch;
    Servo markerMech;
    ModernRoboticsI2cRangeSensor sideSensor1, sideSensor2;

    boolean detected = false;
    GoldAlignDetector detector;

    char pos = 'N';

    @Override
    public void runOpMode() throws InterruptedException {

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
        //sensorSwitch = hardwareMap.servo.get("sensor switch");
        markerMech = hardwareMap.servo.get("marker mech");


        //SENSORS
        MRGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        doubleSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "depot sensor");
        forwardSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "sensor");

        MRGyro.calibrate();
        sleep(4000);

        markerMech.setPosition(0.8);
      //  sensorSwitch.setPosition(.5);

        /*
         * AUTONOMOUS MAIN METHOD
         * The start of our autonomous code
         */

        waitForStart();

        landing();

        sampling();

        crater();

    }

    /*
     * LANDING Method
     * This method allows us to land our robot by detecting our distance
     * from the ground using our MR range sensor. Once we reach the ground
     * we rotate our robot in order to unhook.
     */
    public void landing() {


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

        boolean landed = false;
        while (!landed) {
            if (doubleSensor.getDistance(DistanceUnit.INCH) <= 1.6) {
                sleep(1000);
                liftMotor.setPower(0);
                //GETS US OFF THE HOOK
                liftMotor.setPower(1.0);
                sleep(1900);
                /*
                liftMotor.setPower(-0.8);
                sleep(200);
                */
                liftMotor.setPower(0.0);
              //  move("east", 0.7);
               // sleep(550);
                move("east", 0.5);
                sleep(500);
                gyro(90, "ccw");
                liftMotor.setPower(-1.0);
                sleep(1500);
                liftMotor.setPower(0.0);
                gyro(0, "cw");
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

    /*
     * SAMPLING METHOD
     * This block of code is the logic for our sampling during autonomous
     * with our robot's goal being to find the gold cube.
     * If our partner has already sampled this logic allows us to not
     * circumvent the points gained by their autonomous.
     * The robot begins by 'looking' at the ore through the phones camera
     * The robot checks if the gold cube is in the center first because
     * it is closest to us after landing from the lander.
     * If the detect method is true, which means that the phone has seen the mineral,
     * and the detected variable is false, which means that the robot hasn't already seen the gold,
     * then our robot moves towards the position where these conditions are met.
     */
    public void sampling() {
        move("south", 0.3);
        sleep(300);
        stopMove();
        detect();
        if (detect() && !detected) {
            pos = 'C';
            gyro(12, "ccw");
            telemetry.addData("pos", "center");
            telemetry.update();
            detected = true;
        } else if (!detect() && !detected) {
            gyro(53, "ccw");
            stopMove();
            sleep(400);
            telemetry.addData("pos", "NOT center");
            telemetry.addLine("turning left");
            telemetry.update();
            if (detect() && !detected) {
                pos = 'S';
                telemetry.addData("pos", "left");
                telemetry.update();
                detected = true;
            } else if (!detect() && !detected) {
                gyro(315, "cw");
                stopMove();
                sleep(400);
                telemetry.addData("pos", "NOT left");
                telemetry.addLine("turning right");
                telemetry.update();
                if (detect() && !detected) {
                    pos = 'S';
                    telemetry.addData("pos", "right");
                    telemetry.update();
                    detected = true;
                } else if (!detect() && !detected) {
                    telemetry.addData("pos", "NOT right, giving up");
                    telemetry.update();
                    deposit();
                }
            }
        }
        if (detected) {
            move("south", 0.8);
            sleep(800);
            stopMove();
            telemetry.addLine("we are now moving");
            telemetry.update();
            //sensorSwitch.setPosition(1.25);
            //sensorSwitch.setPosition(1.00);
            //telemetry.addLine("moved forward");
            //telemetry.update();
            deposit();
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
     * DEPOSITING Method
     * This method is used to deposit our
     * team marker into the depot. We use
     * a color sensor to detect the depot
     * box by scanning for the colored
     * tape on the floor.
     */
    public void deposit() {
        // In this portion of the deposit method the robot moves away from the lander
        // and finds its current angle.
        /*
         * We use a switch-case because depending on where the gold mineral was in sampling,
         * we turn a different angle.
         */

        /* The robot moves, using a range sensor to detect its distance from the wall
         * and moves towards it until it detects that it is 10 inches away from it.
         * once it is there, our robot turns left so that we can navigate around the lander bin
         */
      //  sensorSwitch.setPosition(1.0);
        boolean craterCheck = false;
        switch(pos){
            case 'S':
                while(!craterCheck) {
                    if (forwardSensor.getDistance(DistanceUnit.INCH) <= 21) {
                        move("north", 0.8);
                        craterCheck = false;
                    } else if (forwardSensor.getDistance(DistanceUnit.INCH) > 21) {
                        stopMove();
                        craterCheck = true;
                    }
                }
                break;
            case 'C':
                while(!craterCheck) {
                    if (forwardSensor.getDistance(DistanceUnit.INCH) <= 5 ){
                        move("north", 0.8);
                        craterCheck = false;
                    } else if (forwardSensor.getDistance(DistanceUnit.INCH) > 5) {
                        stopMove();
                        craterCheck = true;
                    }
                }
                break;
        }

        telemetry.addLine("moved backwards");
        telemetry.update();
        sleep(500);
        gyro(90, "ccw");
        telemetry.addLine("turned");
        telemetry.update();

        boolean wallcheck = false;
        while (!wallcheck) {
            if (forwardSensor.getDistance(DistanceUnit.INCH) < 13.0) {
                stopMove();
                wallcheck = true;
            } else if (forwardSensor.getDistance(DistanceUnit.INCH) >= 13.0) {
                move("south", 0.9);
                telemetry.addLine("started looking for distance");
                telemetry.addData("distance", doubleSensor.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }
        }

        gyro(135, "ccw");
        /* The robot moves until it is 20 inches away from the depot, at which point
         * it begins to slow down and come to a stop. It then deposits the team marker
         * by rotating our marker mechanism servo
         */

        boolean wallcheck2 = false;
        while (!wallcheck2) {
            if (forwardSensor.getDistance(DistanceUnit.INCH) < 20) {
                stopMove();
                markerMech.setPosition(0.0);
                sleep(500);
                markerMech.setPosition(0.70);
                wallcheck2 = true;
            } else if (forwardSensor.getDistance(DistanceUnit.INCH) >= 20) {
                move("south", 0.9);
                telemetry.addData("distance", doubleSensor.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }
        }
    }

    /*
     * CRATER Method
     * In our crater method we turn our robot
     * so that it is facing the crater and then
     * drive our robot to park.
     */
    public void crater() {
        gyro(325,  "cw");
        /* We again use a while loop in order to check our distance, this time from
         * the edge of the crater. Once we are six inches away, we slow down towards
         * the crater and park our robot on the edge.
         */
        move("south", 0.9);
        sleep(1000);
        boolean wallcheck3 = false;
        while (!wallcheck3) {
            if (forwardSensor.getDistance(DistanceUnit.INCH) < 6.0) {
                stopMove();
                wallcheck3 = true;
            } else if (forwardSensor.getDistance(DistanceUnit.INCH) >= 6.0) {
                move("south", 0.9);
                telemetry.addData("distance", doubleSensor.getDistance(DistanceUnit.INCH));
                telemetry.update();
            }
        }
    }

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
            case "ccw":
                //robot turns clockwise(to the right)
                leftFront.setPower(-speed);
                rightFront.setPower(-speed);
                leftBack.setPower(-speed);
                rightBack.setPower(-speed);
                break;
            case "cw":
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
     * GYRO Method
     * Our gyro method uses the gyro sensor in order
     * to accurately turn our robot. We save a target
     * heading, which is the angle that our robot saves when
     * we calibrate the sensor. We can then compare all
     * subsequent angles to this target heading allowing us
     * to know how much we have rotated. This means that we can
     * make angle-perfect turns such as 90 degrees or 180 degrees.
     * This leaves less of our autonomous to chance and more to
     * the technology behind our gyro sensor.
     * We also have a second parameter so that we can turn both
     * clockwise (cw) and counterclockwise
     */
    public void gyro(int targetHeading, String dir) {
        int heading = MRGyro.getHeading();
        while (heading < targetHeading - 10 || heading > targetHeading + 10) {
            heading = MRGyro.getHeading();

            if (dir.equals("ccw")) {
                move("ccw", 0.5);
            }
            else if (dir.equals("cw")) {
                move("cw", 0.5);
            }

            /*
            if (dir == 'L') {
                move("TURN LEFT", 0.3);
            } else if (dir == 'R') {
                move("TURN RIGHT", 0.3);
            }
            */
            telemetry.addData("heading: ", heading);
            telemetry.addData("target", targetHeading);
            telemetry.update();
        }
        stopMove();

    }

    public void parallel(double dist){
        boolean aligned = false;
        while(!aligned){
            if(sideSensor1.getDistance(DistanceUnit.INCH) > sideSensor2.getDistance(DistanceUnit.INCH)){
                move("CW", 0.3);
            } else if (sideSensor1.getDistance(DistanceUnit.INCH) < sideSensor2.getDistance(DistanceUnit.INCH)){
                move("CCW", 0.3);
            } else {
                telemetry.addLine("good. parallel now.");
                telemetry.update();
                aligned = true;
            }
        }
        boolean correctDist = false;
        while(!correctDist){
            if(sideSensor1.getDistance(DistanceUnit.INCH) >= dist || sideSensor2.getDistance(DistanceUnit.INCH) >= dist){
                move("west", 0.3);
            }else if(sideSensor1.getDistance(DistanceUnit.INCH) <= dist || sideSensor2.getDistance(DistanceUnit.INCH) <= dist){
                move("east", 0.3);
            } else{
                telemetry.addLine("good! correct dist away");
                correctDist = true;
            }
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
