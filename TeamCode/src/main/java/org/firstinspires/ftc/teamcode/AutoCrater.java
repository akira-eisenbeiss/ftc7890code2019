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
/*
7890 Space Lions 2019 "Crater Autonomous for Big Kids"
author: 7890 Software (Akira, Erin, Stephen, Kyra, Anthony)
GOALS: 2019, land, sample, deposit team marker, park in crater
 */


@Autonomous(name="crater auto", group="LinearOpMode")
public class AutoCrater extends LinearOpMode {

    /*
     * MOTORS, SERVOS, and SENSORS
     * In this section of the code, we declare our motors, servos, and sensors
     */
    
    //This code declares the four wheels on our robot:
    DcMotor leftFront, leftBack, rightFront, rightBack;
    
    //The motor for our lift:
    DcMotor liftMotor;
    
    //This code shows the two motors on our double jointed robot arm.
    //Arm Motor 1 is the motor directly attached to the robot
    DcMotor armMotor1, armMotor2;
    
    //The motor on our intake box that is responsible for controlling the intake:
    DcMotor intakeMotor;
    
    //Our range sensor that uses ODS and ultrasonic to detect our distance from objects:
    ModernRoboticsI2cRangeSensor rangeSensor;
    
    //Our gyro sensor that calibrates at a target heading and detects our angle away from that heading
    //We can use this to accurately turn
    ModernRoboticsI2cGyro MRGyro;
    
    //Our color sensor which we use to detect changes in color:
    ModernRoboticsI2cRangeSensor depotSensor;
    // The color sensors tell us where in the field we are
    // We utilize the red and blue tape on the floor as reference points on the field.

    //This servo is used to lock our extending intake.
    Servo lock;

    /*
     * VUFORIA Setup
     * We use VUFORIA to detect the different colored minerals and choose the correct one when sampling.
     */

    private static final String TFOD_MODEL_ASSET = "RoverRuckus.tflite";
    private static final String LABEL_GOLD_MINERAL = "Gold Mineral";
    private static final String LABEL_SILVER_MINERAL = "Silver Mineral";

    private static final String VUFORIA_KEY = "AZhcwsD/////AAABmU6nWBy9q0vlhzyw5qyhapRfHWDsE+hgriYQbJ/Ch7MUKlboZIiyGRBmc5iMadvjcwQAS2hkBE344/ZJqulkhbVXr+gekeDW70w37UFKGoeGRzxfjI/DsOWtlHx3CaAOczXgf7W5cssnDyO/zOoFiYtnRryKuouZnatP/rhl4/DHMqd1jMOWPWhI6+hNwKDdbXVdiwt5xGNTPxbyJ10jXDHt20NNGZSQry4u4/aAOS2hmrSCZgfMG6keAMB2wHj5G+dHL6AKEQwy1NHmBJZwL/O2ZlE+kkTBMSyrUNRXjkGwvSmJcdzSEDYuXGN4VJkVB/Q1CZefS3m+IkLoA75CAjnX4/buslxm3YNs+roKua3v";


    VuforiaLocalizer vuforia;

    private TFObjectDetector tfod;

    boolean runOnce = false;
    int pos;

    private int gold = 0;
    private int silver = 0;
    private int loop = 0;

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

        armMotor1 = hardwareMap.dcMotor.get("arm motor 1");
        armMotor2 = hardwareMap.dcMotor.get("arm motor 2");
        intakeMotor = hardwareMap.dcMotor.get("intake motor");

        //SENSORS
        MRGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        rangeSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "range sensor");
        depotSensor = hardwareMap.get(ModernRoboticsI2cRangeSensor.class, "depot sensor");

        //SERVOS
        lock = hardwareMap.servo.get("lock");

        //VUFORIA
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters(cameraMonitorViewId);

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        }
        else {
            telemetry.addData("sorry", "not compatible with TFOD");
        }
        /*
         * AUTONOMOUS MAIN METHOD
         * The start of our autonomous code
         */

        MRGyro.calibrate();
        waitForStart();
        //landing();
        //GETS US OFF THE HOOK
        gyro(315);
        liftMotor.setPower(0.3);
        sleep(3000);
        gyro(0);

        /*
        //UNLOCKS AND EXTENDS INTAKE, THEN LOCKS IT AGAIN
        lock.setPosition(0.0);
        armMotor1.setPower(1.0);
        armMotor2.setPower(1.0);
        sleep(2000);
        armMotor1.setPower(0.0);
        armMotor2.setPower(0.0);
        lock.setPosition(1.0);
*/
        sampling();
        deposit();
        crater();

    }
        /*
        * LANDING Method
        * This method allows us to land our robot by detecting our distance
        * from the ground using our MR range sensor. Once we reach the ground
        * we rotate our robot in order to unhook.
        */
    public void landing() {
        //cases, naming, data types
        double distanceFromGround = depotSensor.getDistance(DistanceUnit.INCH);
        telemetry.addData("distance", distanceFromGround);
        telemetry.update();
        while (distanceFromGround > 2.5) {
            double landingspeed = 0.3;
            liftMotor.setPower(landingspeed);
        }
        if (distanceFromGround <= 2.8){
            liftMotor.setPower(0);
        }
    }
    /*
     * SAMPLING Method
     * This method uses VUFORIA in order to detect which
     * minerals are silver and which are gold. Our robot
     * uses this information to then rotate and knock the
     * gold mineral out of its starting position.
     */
    public void sampling() {
        if (opModeIsActive()) {
            /** Activate Tensor Flow Object Detection. */
            if (tfod != null) {
                tfod.activate();
            }

            while (opModeIsActive()) {
                if (tfod != null) {
                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.
                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());
                        if (updatedRecognitions.size() == 3) {
                            int goldMineralX = -1;
                            int silverMineral1X = -1;
                            int silverMineral2X = -1;
                            for (Recognition recognition : updatedRecognitions) {
                                if (recognition.getLabel().equals(LABEL_GOLD_MINERAL)) {
                                    goldMineralX = (int) recognition.getLeft();
                                } else if (silverMineral1X == -1) {
                                    silverMineral1X = (int) recognition.getLeft();
                                } else {
                                    silverMineral2X = (int) recognition.getLeft();
                                }
                            }
                            if (goldMineralX != -1 && silverMineral1X != -1 && silverMineral2X != -1) {
                                if (goldMineralX < silverMineral1X && goldMineralX < silverMineral2X) {
                                    //GOLD IS ON LEFT
                                    telemetry.addData("Gold Mineral Position", "Left");
                                    pos = 2;
                                } else if (goldMineralX > silverMineral1X && goldMineralX > silverMineral2X) {
                                    //GOLD IS ON RIGHT
                                    telemetry.addData("Gold Mineral Position", "Right");
                                    pos = 1;
                                } else {
                                    //GOLD IS IN CENTER
                                    telemetry.addData("Gold Mineral Position", "Center");
                                    pos = 0;
                                }
                            }
                            //MOVES BASED OFF OF WHAT WE DETECT
                            if (pos == 1) { //RIGHT
                                gyro(315); //value for testing
                                move(leftFront, rightFront, leftBack, rightBack, "BACKWARDS", 0.3);
                                sleep(3000);
                                move(leftFront, rightFront, leftBack, rightBack, "FORWARDS", 0.3);
                                sleep(3000);
                            } else if (pos == 2) { //LEFT
                                gyro(45); //value for testing
                                move(leftFront, rightFront, leftBack, rightBack, "BACKWARDS", 0.3);
                                sleep(3000);
                                move(leftFront, rightFront, leftBack, rightBack, "FORWARDS", 0.3);
                                sleep(3000);
                            } else if (pos == 0) { //CENTER
                                move(leftFront, rightFront, leftBack, rightBack, "BACKWARDS", 0.3);
                                sleep(5000);
                                move(leftFront, rightFront, leftBack, rightBack, "FORWARDS", 0.3);
                                sleep(3000);
                            } else {
                                telemetry.addData("Error Report", "Error, fix pos va;ue :(");
                            }
                        }
                        telemetry.update();
                    }
                }
            }
        }
        if (tfod != null) {
            tfod.shutdown();
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
        //in this portion of the deposit method the robot backs up from the lander and finds its current angle. 
        MRGyro.calibrate();
        sleep(4000);
        /*
         * We use a switch-case because depending on where the gold ore was in sampling,
         * we have to turn a different angle.
         */
        switch(pos){
            case 0: //CENTER
                gyro(45);
                break;
            case 1: //RIGHT
                gyro(135);
                break;
            case 2: //LEFT
                gyro(45);
                break;
        }
        //the robot checks hwo far it is from the crater
        double distanceValue = rangeSensor.getDistance(DistanceUnit.INCH);

        while(distanceValue > 3){
            //the robot approaches the wall
            move(leftFront, rightFront, leftBack, rightBack, "BACKWARDS", 0.5);
        }
        if (distanceValue <= 3){
           //stops once the robot is close enough to the wall and turns towards the depot
            stop(leftFront, leftBack, rightFront, rightBack);
        }
        switch(pos) {
            case 0: //CENTER
                gyro(135);
                break;
            case 1://RIGHT
                gyro(180);
                break;
            case 2://LEFT
                gyro(90);
                break;
        }
        while(distanceValue > 3){
            //goes towards depot
            move(leftFront, rightFront, leftBack, rightBack, "BACKWARDS", 0.5);
        }
        if (distanceValue <= 3) {
          //stops once the robot is close enough to the depot
            stop(leftFront, leftBack, rightFront, rightBack);
        }
        //unfolds arm, spins intake to deposit
        armMotor1.setPower(0.5);
        sleep(2000);
        armMotor2.setPower(0.5);
        sleep(2000);
        intakeMotor.setPower(0.5);
        sleep(2000);
        //folds and retracts arm
        armMotor1.setPower(-0.5);
        sleep(2000);
        armMotor2.setPower(-0.5);
        sleep(2000);
        intakeMotor.setPower(0.5);
        sleep(2000);
    }
    /*
     * CRATER Method
     * In our crater method we turn our robot
     * so that it is facing the crater and then
     * drive our robot to park.
     */
    public void crater() {
        /*
         * We have a switch-case because we have to turn a different angle depending
         * on where the gold ore is. This is because we reset the gyro sensor's zero at different
         * places depending on where the gold ore was in sampling.
         */
        switch(pos) {
            case 0: //CENTER
                gyro(225);
                break;
            case 1: //RIGHT
                gyro(270);
                break;
            case 2: //LEFT
                gyro(180);
                break;
        }
        double distanceValue = rangeSensor.getDistance(DistanceUnit.INCH);
        while(distanceValue > 6){
            //to turn towards crater
            if(pos == 0){
                gyro(45); //TODO: make sure we are close enough to the wall
                pos = 2;
            }
            else{
                move(leftFront, rightFront, leftBack, rightBack, "BACKWARDS", 0.5);
            }
        }
        if (distanceValue <= 6){
            stop(leftFront, leftBack, rightFront, rightBack);
        }
    }
    /*
    * MOVEMENT Method
    * In this code we use a switch case in order to create
    * more code-efficient robot driving. This means that instead
    * of having to individually control each motor everytime we
    * want to move, we can instead just call the move() method. And specify the wanted direction with a case-valid string.
    * This makes our program a lot shorter than it would
    * normally have been, and makes it easier to program and read our code. It 
    * assigns the proper motor speed assigned when the method is called, and sets the rotation of the motors so that the robot moves in the specified direction
    *(with either positive or negative speed)
    */
    public void move(DcMotor motorlf, DcMotor motorrf, DcMotor motorlb, DcMotor motorrb, String direction, double speed) {
        switch(direction) {
            case "BACKWARDS":
                //robot moves backwards
                motorlf.setPower(-speed);
                motorrf.setPower(-speed);
                motorlb.setPower(speed);
                motorrb.setPower(speed);
                break;
            case "FORWARDS":
                //robot moves forwards
                motorlf.setPower(speed);
                motorrf.setPower(speed);
                motorlb.setPower(-speed);
                motorrb.setPower(-speed);
                break;
            case "RIGHT":
                //robot strafes right
                motorlf.setPower(speed);
                motorrf.setPower(-speed);
                motorlb.setPower(speed);
                motorrb.setPower(-speed);
                break;
            case "LEFT":
                //robot strafes left
                motorlf.setPower(-speed);
                motorrf.setPower(speed);
                motorlb.setPower(-speed);
                motorrb.setPower(speed);
                break;
            case "TURN RIGHT":
                //robot turns clockwise(to the right)
                motorlf.setPower(-speed);
                motorrf.setPower(-speed);
                motorlb.setPower(-speed);
                motorrb.setPower(-speed);
                break;
            case "TURN LEFT":
                //robot turns counterclockwise(to the left)
                motorlf.setPower(speed);
                motorrf.setPower(speed);
                motorlb.setPower(speed);
                motorrb.setPower(speed);
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
     */
    public void gyro(int targetHeading) {
        int heading = MRGyro.getHeading();
        while(heading < targetHeading - 10 || heading > targetHeading + 10) {
            heading = MRGyro.getHeading();

            move(leftFront, leftBack, rightFront, rightBack, "TURN RIGHT", 0.3);
            telemetry.addData("heading: ", heading);
            telemetry.update();
        }
        stop(leftFront, leftBack, rightFront, rightBack);
    }
    /*
    * STOP Method
    * Similar to the move method, our stop method
    * takes in the different wheels as parameters
    * and sets all of their powers to 0, stopping
    * them. This reduces the number of lines needed
    * to stop the robot from four to only one.
    */
    public void stop(DcMotor motorlf, DcMotor motorrf, DcMotor motorlb, DcMotor motorrb) {
        //robot stops moving
        motorlf.setPower(0.0);
        motorrf.setPower(0.0);
        motorlb.setPower(0.0);
        motorrb.setPower(0.0);
    }
    /*
     * VUFORIA Methods
     * This method initializes our VUFORIA using the license key
     * we registered on the vuforia website. The next method,
     * initTFOD, is used to init code that is needed to detect
     * the differences in gold and silver minerals.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier( "tfodMonitorViewId", "Id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);

        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_GOLD_MINERAL, LABEL_SILVER_MINERAL);

    }
}
