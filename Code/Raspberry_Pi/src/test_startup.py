import subprocess

def generate_and_play_audio(text):
    # Please download the binary for Raspberry 4 within Piper's repository.
    # In this case it was already downloaded.
    # It consumes about 40-50% of CPU spike of 500-1000 ms.
    try:
        IsSpeakerActive = True
        # Run Piper to generate audio from the input text and pipe the output directly to aplay
        piper_command = [
            '/home/adrian/Documents/pr_EPES/Code/Raspberry_Pi/src/piper/piper', 
            '-m', '/home/adrian/Documents/pr_EPES/Code/Raspberry_Pi/src/piper/models/en_US-amy-low.onnx', 
            '-c', '/home/adrian/Documents/pr_EPES/Code/Raspberry_Pi/src/piper/models/en_US-amy-low.onnx.json', 
            '-s', '0', 
            '-f', '-'
        ]

        
        # Use subprocess to pipe the text into Piper and then directly to aplay
        piper_process = subprocess.Popen(piper_command, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
        aplay_process = subprocess.Popen(['aplay'], stdin=piper_process.stdout)

        # Write the text to Piper's stdin and then wait for aplay to finish
        piper_process.stdin.write(text.encode())  # Pass text as bytes to Piper
        piper_process.stdin.close()  # Close stdin after writing
        aplay_process.wait()  # Wait for aplay to finish

        # Check if both processes completed successfully
        print("DONE")
        return IsSpeakerActive  # Indicate success

    except Exception as e:
        print(f"Error occurred: {e}")
        IsSpeakerActive = False
        return IsSpeakerActive  # Return False if an exception occurs


if __name__ == "__main__":
	generate_and_play_audio(f"Booting Completed")
