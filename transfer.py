import os

from ssh_transfer import create_ssh_client_with_cfg_file, transfer_file



main_folder="./tasks"

task_folders=[f for f in os.listdir(main_folder) if os.path.isdir(os.path.join(main_folder, f))]

client=create_ssh_client_with_cfg_file("ssh_config.json")

for task_folder in task_folders:
    task_folder_path=os.path.join(main_folder, task_folder)
    color_image_path=os.path.join(task_folder_path, "combined_image.png")
    remote_folder="~/code/Ben-VLM/datasets/real_world_test_images/"
    color_image_remote_path = os.path.join(remote_folder, f"{task_folder}_image.png")  # Path on the server

    transfer_file(client, color_image_path, color_image_remote_path)
    print(f"The color image in folder: {task_folder} transferred to the server")