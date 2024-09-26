import paramiko
from scp import SCPClient
import os
import json
import argparse

from image_capture import capture_image_from_depth_camera


# Function to create an SSH client
def create_ssh_client(hostname, port, username, password):
    """Create an SSH client."""
    client = paramiko.SSHClient()
    client.load_system_host_keys()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(hostname, port=port, username=username, password=password)
    return client

def create_ssh_client_with_cfg_file(cfg_file):
    if not os.path.exists(cfg_file) or not cfg_file.endswith(".json"):
        print("Config file does not exist or is not a JSON file.")
        return None

    with open (cfg_file) as f:
        data=json.load(f)
        hostname=data["hostname"]
        port=data["port"]
        username=data["username"]
        password=data["password"]

    client=create_ssh_client(hostname, port, username, password)

    return client
# Function to transfer the image using SCP
def transfer_file(ssh_client, local_file_path, remote_path):
    """Transfer the image using SCP."""
    with SCPClient(ssh_client.get_transport()) as scp:
        scp.put(local_file_path, remote_path)
        print(f"File {local_file_path} transferred to {remote_path}")




if __name__ == "__main__":


    

    save_folder="./pick_place_task_1"
    remote_folder="~/code/Ben-VLM/datasets/real_world_test_images/"
    color_image_remote_path = os.path.join(remote_folder,"test_combined_image.png")  # Path on the server

    # Capture the depth image

    color_image_path,_,_=capture_image_from_depth_camera(save_folder)

    # Create SSH client
    ssh_client=create_ssh_client_with_cfg_file("ssh_config.json")

    # Transfer the image to the remote server
    transfer_file(ssh_client, color_image_path, color_image_remote_path)

    # Close the SSH connection
    ssh_client.close()
