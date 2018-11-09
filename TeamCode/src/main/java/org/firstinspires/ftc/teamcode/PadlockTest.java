package org.firstinspires.ftc.teamcode;

/**
 * Created by wenhuizhao on 11/8/18.
 */
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="padlock test", group="Tele Op")
public class PadlockTest extends OpMode{
    private ElapsedTime runtime = new ElapsedTime();
    CRServo padLock;

    @Override
    public void init() {
        padLock = hardwareMap.crservo.get("padlock");
    }

    @Override
    public void loop() {
        boolean gamepad2A = gamepad2.a;
        boolean gamepad2B = gamepad2.b;
        boolean gamepad2X = gamepad2.x;

        int servo = 0;
        //ASSIGN TO BUTTONS
     /*   if (gamepad2B) { //close padlock
            servo = 1;
        }
        else if (gamepad2A) { //open padlock
            servo = 2;
        }
        else if (gamepad2X){
            servo = 0;
        }*/

        //CODE FOR PRESSING BUTTONS
    //    if(servo == 0){
      //      padLock.setPower(-0.0001);
        //}
      /*  else if (servo == 1){
            padLock.setPower(-1);
        }
        else if (servo == 2) {
            padLock.setPower(1);
        }*/
        padLock.setPower(0.5);
    }


}


