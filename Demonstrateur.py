import NiRyo_ChaikMat_Ecosyspro as rob
import ChaikmatWLKATA as willie
import os, random, time
import multitasking

# Initialize robots
chaikmat_ned = rob.ChaikNiRyo()
chaikmat_ned.setup()
chaikmat_wl = willie.ChaikWLKATA()

# AAS File Paths
ned_aas_file = "~/Runchain_Services/NiryoNed2AAS.aas.xml"
wl_aas_file = "~/Runchain_Services/WlkataMirobotAAS.aas.xml"

arch_file_path = "/media/chaikmat/4621-0000/arch.txt"

def pick_replace():
    # Update robot state to Active before operation
    chaikmat_ned.robot_state_update(ned_aas_file, "Active")
    
    # Check and perform Load Piece service for NiRyo
    if "Error" in chaikmat_ned.service_query(ned_aas_file, "Pick"):
        print("Service 'Pick' not available in NiRyo AAS.")
        chaikmat_ned.robot_state_update(ned_aas_file, "Idle")
        return
    chaikmat_ned.load_piece()

    # Check and perform Convey Until Detect service for NiRyo
    if "Error" in chaikmat_ned.service_query(ned_aas_file, "Convey"):
        print("Service 'Convey' not available in NiRyo AAS.")
        chaikmat_ned.robot_state_update(ned_aas_file, "Idle")
        return
    chaikmat_ned.convey_until_detect()

    # Check and perform Color and Shape Detection service for NiRyo
    if "Error" in chaikmat_ned.service_query(ned_aas_file, "ColorAndShapeDetection"):
        print("Service 'ColorAndShapeDetection' not available in NiRyo AAS.")
        chaikmat_ned.robot_state_update(ned_aas_file, "Idle")
        return
    available = chaikmat_ned.object_chars()
    print(available)

    # Check and perform Vision Pick service for NiRyo
    if "Error" in chaikmat_ned.service_query(ned_aas_file, "Pick"):
        print("Service 'Pick' not available in NiRyo AAS.")
        chaikmat_ned.robot_state_update(ned_aas_file, "Idle")
        return
    chaikmat_ned.vpick()

    # Check and perform Place service for NiRyo
    if "Error" in chaikmat_ned.service_query(ned_aas_file, "Place"):
        print("Service 'Place' not available in NiRyo AAS.")
        chaikmat_ned.robot_state_update(ned_aas_file, "Idle")
        return
    chaikmat_ned.put_back_piece()

    # Update robot state to Idle after operation
    chaikmat_ned.robot_state_update(ned_aas_file, "Idle")

def vision_test():
    chaikmat_ned.robot_state_update(ned_aas_file, "Active")

    if "Error" in chaikmat_ned.service_query(ned_aas_file, "ColorAndShapeDetection"):
        print("Service 'ColorAndShapeDetection' not available in NiRyo AAS.")
        chaikmat_ned.robot_state_update(ned_aas_file, "Idle")
        return
    chaikmat_ned.select_object()

    chaikmat_ned.robot_state_update(ned_aas_file, "Idle")

def parse_arches(file_path):
    arches = {}
    with open(file_path, 'r') as arch_file:
        lines = arch_file.readlines()
    
    current_key = ""
    for line in lines:
        if line.endswith(':\n'):
            current_key = line[:-2]  # Remove the colon and newline
            arches[current_key] = {}
        elif 'shape:' in line:
            shape = line.split(': ')[1].strip()
            arches[current_key]['shape'] = shape
        elif 'color:' in line:
            color = line.split(': ')[1].strip()
            arches[current_key]['color'] = color
    print(arches)
    return arches

@multitasking.task
def build_arches(file, arches):
    print("start building")
    if file != "nofile":
        with open(arch_file_path, 'r') as f:
            lines = f.readlines()
            for line in lines:
                print(line.strip())
    
    number = -1
    for arch in arches:
        number += 1
        found = 0
        chaikmat_ned.neutral()
        shape = arches[arch].get('shape')
        color = arches[arch].get('color')
        while found == 0:
            chaikmat_ned.robot_state_update(ned_aas_file, "Active")

            if "Error" in chaikmat_ned.service_query(ned_aas_file, "Pick"):
                print("Service 'Pick' not available in NiRyo AAS.")
                chaikmat_ned.robot_state_update(ned_aas_file, "Idle")
                return
            chaikmat_ned.load_piece()

            if "Error" in chaikmat_ned.service_query(ned_aas_file, "Convey"):
                print("Service 'Convey' not available in NiRyo AAS.")
                chaikmat_ned.robot_state_update(ned_aas_file, "Idle")
                return
            chaikmat_ned.convey_until_detect()

            if "Error" in chaikmat_ned.service_query(ned_aas_file, "Pick"):
                print("Service 'Pick' not available in NiRyo AAS.")
                chaikmat_ned.robot_state_update(ned_aas_file, "Idle")
                return
            found = chaikmat_ned.pick_my_thing(shape, color, number)

        chaikmat_ned.robot_state_update(ned_aas_file, "Idle")
    chaikmat_ned.neutral()
    chaikmat_ned.close_xml()

    chaikmat_wl.robot_state_update(wl_aas_file, "Active")

    # Check and perform Move Conveyor service for Wlkata
    if "Error" in chaikmat_wl.service_query(wl_aas_file, "Move"):
        print("Service 'Move' not available in Wlkata AAS.")
        chaikmat_wl.robot_state_update(wl_aas_file, "Idle")
        return
    chaikmat_wl.move_conveyor(100)

    chaikmat_wl.robot_state_update(wl_aas_file, "Idle")

@multitasking.task
def construct(arches):
    time.sleep(5)
    build_arches("nofile", arches)

# Set empirical values
chaikmat_ned.robot.set_brightness(0.78)
chaikmat_ned.robot.set_contrast(1.5)
chaikmat_ned.robot.set_saturation(0.7)

userchoice = "None"
while userchoice != "q":
    userchoice = input("\n\nredefine setup: s \nNew Workspace: w \npicking and replace: p \nDefine arches building points: b\nDefine and build new Arch: n \nLoad and build arch from file: l\nTest vision: v\nquit: q\nYour choice? ")
    if userchoice == "p":
        pick_replace()
    elif userchoice == "s":
        chaikmat_ned.reconfigure()
    elif userchoice == "v":
        vision_test()
    elif userchoice == "n":
        arches = parse_arches(arch_file_path)
        construct(arches)
    elif userchoice == "l":
        arches = parse_arches(arch_file_path)
        build_arches("nofile", arches)
    elif userchoice == "w":
        chaikmat_ned.new_workspace()
    elif userchoice == "b":
        chaikmat_ned.define_arches_points()
    elif userchoice == "q":
        print("Quitting program.")
    else:
        print("Unknown command:", userchoice)

    # Example of dynamically configuring a service (Commented out):
    # chaikmat_ned.configure_service(ned_aas_file, "NewService", "input_value", "output_value", "driver_function", "effector")
    # chaikmat_wl.configure_service(wl_aas_file, "NewService", "input_value", "output_value", "driver_function", "effector")

    # Example of dynamically adding a service (Commented out):
    # chaikmat_ned.add_service(ned_aas_file, "NewService", "input_value", "output_value", "driver_function", "effector")
    # chaikmat_wl.add_service(wl_aas_file, "NewService", "input_value", "output_value", "driver_function", "effector")

    # Example of dynamically removing a service (Commented out):
    # chaikmat_ned.remove_service(ned_aas_file, "ServiceToRemove")
    # chaikmat_wl.remove_service(wl_aas_file, "ServiceToRemove")
