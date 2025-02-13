import time

class TimedExecutor:
    def __init__(self, interval_ms):
        """
        Initialize the TimedExecutor with a specific interval.

        Args:
            interval_ms (int): The interval in milliseconds between executions.
        """
        self.interval_ms = interval_ms
        self.last_execution_time = 0  # Track the last execution time

    def execute(self, ser, data):
        """
        Executes `ser.write()` if the specified interval has elapsed.

        Args:
            ser: The serial connection object.
            data: The data to send (bytes or string).
        """
        current_time = int(time.time() * 1000)  # Current time in milliseconds
        if current_time - self.last_execution_time >= self.interval_ms:
            self.last_execution_time = current_time  # Update the last execution time
            ser.write(data.encode("utf-8") if isinstance(data, str) else data)
