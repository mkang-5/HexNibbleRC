package org.firstinspires.ftc.teamcode._MecanumTestRobot;

import org.hexnibble.corelib.misc.Constants;
import org.hexnibble.corelib.misc.Msg;

public class MTConstants extends Constants {

   public MTConstants() {
      Msg.log(getClass().getSimpleName(), "Constructor", "Setting USE_FTCONTROL_DASHBOARD to true");
      USE_FTCONTROL_DASHBOARD = true;
   }
}
