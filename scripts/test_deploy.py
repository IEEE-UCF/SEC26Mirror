import os
import datetime


folder_path = "log"
os.makedirs(folder_path, exist_ok=True)

def create_status_file():
    # This will create the file in the root of the repo (parent of scripts/)
    # because the Action runs from the repo root.
    filename = "log/gitea_verification.txt"
    
    time_now = datetime.datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    # Get the hostname to prove it's the Pi
    hostname = os.popen("hostname").read().strip()
    
    content = (
        f"✅ Gitea Action Verification\n"
        f"---------------------------\n"
        f"Executed on: {hostname}\n"
        f"Time:        {time_now}\n"
        f"Script Path: scripts/test_deploy.py\n"
        f"Status:      SUCCESS\n"
    )

    with open(filename, "w") as f:
        f.write(content)
    
    print(f"--- SUCCESS ---")
    print(f"Created {filename} on {hostname}")
    print(f"File contents:\n{content}")

if __name__ == "__main__":
    create_status_file()