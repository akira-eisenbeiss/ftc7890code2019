package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//MOTORS + SERVOS
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.CRServo;

//SENSORS
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cRangeSensor;
import com.qualcomm.robotcore.hardware.OpticalDistanceSensor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.hardware.rev.Rev2mDistanceSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


@Autonomous(name="AUTO DEPOT 1", group="LinearOpMode")
public class AutoDepotSide1 extends LinearOpMode {
    private ElapsedTime     runtime = new ElapsedTime();

    //test
    //motors
    DcMotor leftFront, rightFront, leftBack, rightBack;
    DcMotor armMotor;
    DcMotor liftMotor;

    //SERVOS
    Servo markerMech;

    //sensors
    /*
   two dist sensors
   -gyro sensor
     */
    //IMU
    BNO055IMU imu;
    //ANGLES
    Orientation angle;
    Acceleration gravity;
    Orientation lastAngle = new Orientation();
    //DISTANCE SENSORS
    DistanceSensor rangeSensorBottom, rangeSensorFront;
    //COLOR SENSOR
    ColorSensor depotSensor;

    static final double     COUNTS_PER_MOTOR_REV    = 1440 ;    // eg: TETRIX Motor Encoder
    static final double     DRIVE_GEAR_REDUCTION    = 2.0 ;     // This is < 1.0 if geared UP
    static final double     WHEEL_DIAMETER_INCHES   = 4.0 ;     // For figuring circumference
    static final double     COUNTS_PER_INCH         = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * 3.1415);
    static final double     DRIVE_SPEED             = 0.5;
    static final double     TURN_SPEED              = 0.5;

    //VALUE FOR LANDING (TESTING)
    double landingValue = 0.5;

    public void runOpMode() {

        leftFront = hardwareMap.dcMotor.get("left front");
        rightFront = hardwareMap.dcMotor.get("right front");
        leftBack = hardwareMap.dcMotor.get("left back");
        rightBack = hardwareMap.dcMotor.get("right back");
        armMotor = hardwareMap.dcMotor.get("arm motor");
        liftMotor = hardwareMap.dcMotor.get("lift motor");
/*
        markerMech = hardwareMap.servo.get("marker");
        */
/*
        //COLOR AND RANGE SENSORS
        depotSensor = hardwareMap.get(ColorSensor.class, "depot sensor");
        rangeSensorBottom = hardwareMap.get(DistanceSensor.class, "range sensor bottom");
        rangeSensorFront = hardwareMap.get(DistanceSensor.class, "range sensor front");
        */
        //SETTING UP REV GYRO (IMU)
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode                = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit           = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit           = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled      = false;
        parameters.loggingTag          = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        while (!isStopRequested() && !imu.isGyroCalibrated())
        {
            sleep(50);
            idle();
        }

        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);
        while(opModeIsActive()) {
            landing();
            depositBlue();
            //isImuCorrect

        }
    }

    public void landing() {
        double distanceFromGround = rangeSensorBottom.getDistance(DistanceUnit.CM);
        while (distanceFromGround > 6) { //this value is for testing
            liftMotor.setPower(landingValue);
        }
        if (distanceFromGround == 6) {
            liftMotor.setPower(-landingValue); //basically just rewinds the motor so that we can retract the lift
        }
    }

    public void depositBlue() {
        while(!(depotSensor.blue() > depotSensor.green()) && !(depotSensor.blue() > depotSensor.red())){
            move(leftFront, rightFront, leftBack, rightBack, true);
        }
        depositing();
    }
    public void depositRed() {
        while(!(depotSensor.red() > depotSensor.green()) && !(depotSensor.red() > depotSensor.blue())){
            move(leftFront, rightFront, leftBack, rightBack, true);
        }
        depositing();
    }

    public int isImuCorrect(double imuTarget, double imuRange, double imuActual, double speed) {
        int correctCount = 0;
        double delta = (imuTarget - currentAngle() + 360.0) % 360.0; //HOW MUCH WE NEED TO TURN
        if (delta > 180)
            delta -= 360;
        if (Math.abs(delta) > imuRange) {
            correctCount = 0;
            double imuMod = delta / 45.0; //IF delta > 45, GETS SOMETHING GREATER THAN 1 OR -1
            if (Math.abs(imuMod) > 1.0){
                imuMod = Math.signum(imuMod); //set gyromod to 1 or -1 if error is more than 45 degrees
            }
            turning(speed * Math.signum(imuMod));
        }
        else {
            correctCount++;
            turning(0.0);
        }
        return correctCount;
    }

    public void turning(double speed) {
        angle   = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        gravity  = imu.getGravity();
        /*
        MAKES IT SO THAT POSITIVE SPEEDS MAKE US TURN CLOCKWISE,
        NEGATIVE SPEEDS MAKE US TURN COUNTERCLOCKWISE
         */
        leftFront.setPower(speed);
        leftBack.setPower(speed);
        rightFront.setPower(-speed);
        rightBack.setPower(-speed);
    }

    public double currentAngle() {
        double globalAngle = 0;
        double deltaAngle = angle.firstAngle - lastAngle.firstAngle;

        //MAKES CHANGE IN ANGLE BETWEEN -180 AND 180
        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngle = angle;

        return globalAngle;

    }

    public void depositing() {
        //TODO: test servo, preferably w/ test code
        markerMech.setPosition(0.5); //value for testing
    }

    public void move(DcMotor motorlf, DcMotor motorrf, DcMotor motorlb, DcMotor motorrb, boolean direction) {
        //ONE METHOD TO MOVE FORWARDS AND BACKWARDS
        //values for testing
        if (!direction) {
            motorlf.setPower(-.3);
            motorrf.setPower(-.3);
            motorlb.setPower(-.3);
            motorrb.setPower(-.3);
        }
        else if(direction) {
            motorlf.setPower(.3);
            motorrf.setPower(.3);
            motorlb.setPower(.3);
            motorrb.setPower(.3);
        }
    }
}
