package org.firstinspires.ftc.teamcode.AutonFinite;

import org.firstinspires.ftc.teamcode.CompBotW2.CompBotW2Attachments;

public class DriveObject {
    CompBotW2Attachments r;

    public DriveObject(CompBotW2Attachments r) {
        this.r = r;
    }
    public void driveFinish() {
        r.fl.setPower(0);
        r.fr.setPower(0);
        r.bl.setPower(0);
        r.br.setPower(0);
        r.useEncoders(); // Switch back to normal RUN_USING_ENCODERS velocity control mode
    }
}
