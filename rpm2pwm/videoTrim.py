import os
import subprocess

def trim_videos(input_folder, output_folder, duration=10):
    if not os.path.exists(output_folder):
        os.makedirs(output_folder)
    
    for filename in os.listdir(input_folder):
        if filename.endswith('.mp4'):
            input_path = os.path.join(input_folder, filename)
            output_path = os.path.join(output_folder, f"trimmed_{filename}")
            
            command = [
                "ffmpeg", "-i", input_path, "-t", str(duration),
                "-c", "copy", output_path, "-y"
            ]
            
            subprocess.run(command, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
            print(f"Trimmed: {filename} -> {output_path}")

if __name__ == "__main__":
    trim_videos("./videos", "./videos")