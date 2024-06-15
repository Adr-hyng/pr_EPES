
def run_tts(args):
    import subprocess
    try:
        text = args
        # Run the C++ executable for each text input
        process = subprocess.run(['./ttsDemo'], input=text, text=True, capture_output=True)
        print(process.stdout)
        if process.returncode != 0:
            print(f"Error: {process.stderr}")
    except KeyboardInterrupt:
        print("\nExiting...")

def is_process_running(process_name):
    import psutil
    """
    Check if there is any running process that contains the given name process_name.
    """
    for proc in psutil.process_iter(['pid', 'name', 'cmdline']):
        try:
            # Check if process name or command line matches the given process_name
            if process_name in proc.info['name'] or process_name in " ".join(proc.info['cmdline']):
                return True
        except (psutil.NoSuchProcess, psutil.AccessDenied, psutil.ZombieProcess):
            pass
    return False


if __name__ == "__main__":
    # ~ run_tts("Hello, World")
    process_name = "ttsDemo"
    if is_process_running(process_name):
        print(f"{process_name} is running.")
    else:
        print(f"{process_name} is not running.")
