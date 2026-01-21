import os
import shutil
import subprocess
import argparse
from pathlib import Path

def organize_sd_card(sd_path, cli_path):
    sd_root = Path(sd_path).resolve()
    cli_tool = Path(cli_path).resolve()

    if not sd_root.exists():
        print(f"Error: SD card path '{sd_root}' does not exist.")
        return

    if not cli_tool.exists():
        print(f"Error: CLI tool '{cli_tool}' not found.")
        return

    patterns_dir = sd_root / "patterns"
    patterns_dir.mkdir(exist_ok=True)

    # Find all .thr files in the root or subdirectories (flat scan)
    # We'll move them, so we iterate a list first
    thr_files = list(sd_root.rglob("*.thr"))

    print(f"Found {len(thr_files)} THR files.")

    for thr_file in thr_files:
        # Skip if already in the target structure to avoid re-processing
        # Target structure: patterns/<stem>/<stem>.thr
        if thr_file.parent.parent == patterns_dir and thr_file.name == f"{thr_file.parent.name}.thr":
            print(f"Skipping already organized: {thr_file.name}")
            continue

        name = thr_file.stem
        target_dir = patterns_dir / name
        target_dir.mkdir(exist_ok=True)

        target_thr = target_dir / f"{name}.thr"
        target_png = target_dir / f"{name}.png"
        target_gif = target_dir / f"{name}.gif"

        # Move THR file
        print(f"Processing {name}...")
        shutil.move(str(thr_file), str(target_thr))

        # Generate PNG and GIF using CLI
        # ./ThrGenCLI vis <input> --png <png> --gif <gif>
        cmd = [
            str(cli_tool),
            "vis",
            str(target_thr),
            "--png", str(target_png),
            "--gif", str(target_gif),
            "--size", "512" # Reasonable size for previews
        ]

        try:
            subprocess.run(cmd, check=True, capture_output=True)
        except subprocess.CalledProcessError as e:
            print(f"Failed to generate previews for {name}: {e}")
            # Move back? No, keep it there, user can fix.

    print("Organization complete.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Organize Sisyphus SD card and generate previews.")
    parser.add_argument("sd_path", help="Path to the SD card root")
    parser.add_argument("--cli", default="./build/ThrGenCLI", help="Path to ThrGenCLI executable")
    
    args = parser.parse_args()
    
    organize_sd_card(args.sd_path, args.cli)
