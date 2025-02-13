import os
import subprocess

class RaspberryPiManager:
    """A class to manage Raspberry Pi operations like restart and shutdown."""

    @staticmethod
    def restart():
        """Restarts the Raspberry Pi using the 'sudo shutdown -r now' command."""
        try:
            # Check if the script is running with sufficient privileges
            if os.geteuid() != 0:
                raise PermissionError("This function requires root privileges. Run the script with 'sudo'.")

            print("Restarting the Raspberry Pi...")
            # Execute the restart command
            subprocess.run(["sudo", "shutdown", "-r", "now"], check=True)
        except PermissionError as e:
            print(f"Error: {e}")
        except subprocess.CalledProcessError as e:
            print(f"Failed to restart. Error: {e}")
        except Exception as e:
            print(f"An unexpected error occurred: {e}")

    @staticmethod
    def shutdown():
        """Shuts down the Raspberry Pi using the 'sudo shutdown now' command."""
        try:
            # Check if the script is running with sufficient privileges
            if os.geteuid() != 0:
                raise PermissionError("This function requires root privileges. Run the script with 'sudo'.")

            print("Shutting down the Raspberry Pi...")
            # Execute the shutdown command
            subprocess.run(["sudo", "shutdown", "-h", "now"], check=True)
        except PermissionError as e:
            print(f"Error: {e}")
        except subprocess.CalledProcessError as e:
            print(f"Failed to shut down. Error: {e}")
        except Exception as e:
            print(f"An unexpected error occurred: {e}")
