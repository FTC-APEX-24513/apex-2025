package edu.exeter.apex.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Outtake {
    private DcMotor outtakeMotor;

    public  Outtake(HardwareMap hardwareMap) {
        outtakeMotor = hardwareMap.get(DcMotor.class, "intake");
        outtakeMotor.setZeroPowerBehavior((DcMotor.ZeroPowerBehavior.BRAKE));
    }

    public void outtake(){
        outtakeMotor.setPower(0.9);
    }


}
