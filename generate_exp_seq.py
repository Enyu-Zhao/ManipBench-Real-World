import os
import json
import random
import sys
import argparse

VLM_LIST=[
    "gpt-4o",
    "gpt-4o-mini",
    "glm",
    "internvl",
    "gemini-1.5-pro",
    "gemini-1.5-flash",
]


    


def generate_exp_seq(**kwargs):
    with open("vlm_list.json",'r') as f:
        vlm_info_list=json.load(f)

    
    idxs=[i for i in range(len(vlm_info_list.keys()))]
    random.shuffle(idxs)
    model_list=[]

    for idx in idxs:
        model_name=vlm_info_list[str(idx)]['model_name']
        model_list.append(model_name)

    print(f"Model tested sequence: {model_list}")

    for i,model_idx in enumerate(idxs):
        model=vlm_info_list[str(model_idx)]['model_name']

        print(f"Testing model: {model}")
        input("Prepare the clapper board. Press enter to continue...")
        print(f"-----------------------------------\n")
        if i==0:
            os.system(f"""python moka_real_world_pipeline.py --task_name {kwargs['task_name']} --version {kwargs['version']} --model_name {model} --test_gripper --moving_height {kwargs["moving_height"]} --place_height {kwargs["place_height"]}""")

        else:
            os.system(f"""python moka_real_world_pipeline.py --task_name {kwargs['task_name']} --version {kwargs['version']} --model_name {model} --moving_height {kwargs["moving_height"]} --place_height {kwargs["place_height"]}""")

        
        print(f"Test for {model} completed. Please reinitialize and press enter when ready\n")

        success=input("Record whether the test was successful or not. Enter 'y' for success and 'n' for failure: ")

        if success=='n'or success.lower()=='n':
            success=False
        elif success=='y' or success.lower()=='y':
            success=True
        else:
            print("Invalid input. Record the test as failed.")
            success=False




        vlm_info_list[str(model_idx)]['task_tested'].append({f"""{kwargs['task_name']}_{kwargs["version"]}""":success})


        

        print(f"-----------------------------------\n")


    with open("vlm_list.json",'w') as f:
        json.dump(vlm_info_list,f,indent=4)

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Generate experiment sequence for VLM models.")
    parser.add_argument("--task_name", type=str, default="pick_place_task_1", help="Name of the task.")
    parser.add_argument("--version", type=int, default=0, help="Version of the task.")
    parser.add_argument("--moving_height", type=float, default=0.2, help="Height for moving the object.")
    parser.add_argument("--place_height", type=float, default=0.05, help="Height for placing the object.")

    args = parser.parse_args()

    generate_exp_seq(**vars(args))







