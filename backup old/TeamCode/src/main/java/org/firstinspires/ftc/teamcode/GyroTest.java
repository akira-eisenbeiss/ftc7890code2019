package org.firstinspires.ftc.teamcode;

/**
 * Created by wenhuizhao on 12/8/18.
 */

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cGyro;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.IntegratingGyroscope;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

@Autonomous(name="gyro test", group="LinearOpMode")
public class GyroTest extends LinearOpMode {

    ModernRoboticsI2cGyro MRGyro;
    IntegratingGyroscope gyro;

    public void runOpMode() {
        MRGyro = hardwareMap.get(ModernRoboticsI2cGyro.class, "gyro");
        gyro = (IntegratingGyroscope) MRGyro;

        MRGyro.calibrate();

        waitForStart();
        while(opModeIsActive()){
            int heading = MRGyro.getHeading();
            telemetry.addData("heading: ", heading);
            telemetry.update();
        }

    }
}