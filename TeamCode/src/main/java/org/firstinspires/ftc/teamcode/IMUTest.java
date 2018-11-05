package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;


/**
 * Created by wenhuizhao on 11/4/18.
 */
@Autonomous(name="IMU Test", group="LinearOpMode")
public class IMUTest extends LinearOpMode {
    private ElapsedTime     runtime = new ElapsedTime();

    DcMotor leftFront, rightFront, leftBack, rightBack;
    //IMU
    BNO055IMU imu;
    //ANGLES
    Orientation angle;
    Acceleration gravity;
    Orientation lastAngle = new Orientation();

    public void runOpMode() {
        leftFront = hardwareMap.dcMotor.get("left front");
        rightFront = hardwareMap.dcMotor.get("right front");
        leftBack = hardwareMap.dcMotor.get("left back");
        rightBack = hardwareMap.dcMotor.get("right back");

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.loggingEnabled = false;
        parameters.loggingTag = "IMU";
        parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();
        imu.initialize(parameters);

        while (!isStopRequested() && !imu.isGyroCalibrated()) {
            sleep(50);
            idle();
        }

        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        while(opModeIsActive()) {
            isImuCorrect(90, 10, .3);
        }
    }
        public int isImuCorrect(double imuTarget, double imuRange,/* double imuActual,*/ double speed) {
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
}
