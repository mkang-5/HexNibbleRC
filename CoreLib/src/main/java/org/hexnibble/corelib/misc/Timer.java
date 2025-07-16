package org.hexnibble.corelib.misc;

public class Timer {
  public enum TimerUnit {
    s,
    ms
  }

  protected long startTime;
  protected long lapTime;

  /** Creates a timer and starts running it immediately. */
  public Timer() {
    startTime = System.currentTimeMillis();
  }

  /**
   * Return the elapsed time since the timer was started.
   *
   * @param unit Unit of time desired (s, ms)
   * @return Time since timer was reset
   */
  public long getElapsedTime(Timer.TimerUnit unit) {
    if (unit == TimerUnit.s) {
      return (System.currentTimeMillis() - startTime) / 1000L;
    }
    else {
      return System.currentTimeMillis() - startTime;
    }
//    return switch (unit) {
//      case s -> ((System.currentTimeMillis() - startTime) / 1000L);
//      case ms -> System.currentTimeMillis() - startTime;
//    };
  }

  /**
   * Return the elapsed time in milliseconds since the timer was started.
   *
   * @return Time since timer was reset in milliseconds
   */
  public long getElapsedTime_ms() {
    return System.currentTimeMillis() - startTime;
  }

  public void setLapTime() {
    lapTime = System.currentTimeMillis();
  }

  public long getLapTime() {
    return System.currentTimeMillis() - lapTime;
  }

  public void restartTimer() {
    startTime = System.currentTimeMillis();
  }
}
