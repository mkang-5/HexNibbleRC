package org.firstinspires.ftc.teamcode._MecanumTestRobot;

import org.hexnibble.corelib.exception.InvalidArgumentException;
import org.hexnibble.corelib.misc.ConfigFile;

public class MTConfigFile extends ConfigFile {


   /**
    * Constructor. The hard-coded path for the config file is /sdcard/FIRST Initialize hard-coded
    * constants here in the constructor.
    *
    * @param configFileName Name of the config file.
    */
   public MTConfigFile(String configFileName) {
      super(configFileName);
   }

   /**
    * This method is called during initialization to read in the constants from the config file.
    * Remember to call the parent method as well at some point to initialize the constants defined in
    * the library.
    */
   @Override
   protected void readConfigFile() throws InvalidArgumentException {
      super.readConfigFile();


   }
}
