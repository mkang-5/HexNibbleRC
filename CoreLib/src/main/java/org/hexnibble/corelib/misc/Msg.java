package org.hexnibble.corelib.misc;

public final class Msg {

  public static void log(String className, String functionName, String logString) {
    android.util.Log.i(Constants.TAG, className + ".j: " + functionName + "()- " + logString);
  }

  public static void log(String logString) {
    android.util.Log.i(Constants.TAG, logString);
  }

  public static void logIfDebug(String className, String functionName, String logString) {
    if (ConfigFile.DEBUG_MODE) {
      android.util.Log.i(Constants.TAG, className + ".j: " + functionName + "()- " + logString);
    }
  }
}
