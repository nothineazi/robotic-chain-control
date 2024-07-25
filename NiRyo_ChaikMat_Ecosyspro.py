from pyniryo import *
import os, time ,ast 
from datetime import datetime 
import xml.etree.ElementTree as ET
import asyncio
from client import get_pressure

# Define the AAS file path for Wlkata
ned_aas_file = "~/Runchain_Services/NiryoNed2AAS.aas.xml"


class ChaikNiRyo:
    def __init__(self):
        # Set the default IP for the robot
        self.ip = "10.10.10.10"
        # Set the default sensor pin
        self.sensor = PinID.DI5
        self.wp_name = "10Jul2024_14h18"
        stamp = str(time.time())
        self.logfile = open("logfile" + stamp + ".txt", 'w')
        self.safe_pickpoint = None
        self.pickpoint = None
        self.arches_points = [None, None, None, None]
        self.observe_point = None
        self.reload_point = None
        self.conveyor_starting_point = None
        self.build_point = None


    def read_coordinates_from_file(self, file_path):
        coordinates_dict = {}
        current_point_name = ""
        current_point_values = []

        try:
            with open(file_path, "r") as file:
                for line in file:
                    line = line.strip()
                    if line:
                        if ':' in line:
                            if current_point_name:
                                coordinates_dict[current_point_name] = current_point_values
                            current_point_name, value = line.split(":", 1)
                            current_point_name = current_point_name.strip()
                            current_point_values = []
                            values = value.split(",")
                            for val in values:
                                _, v = val.split("=")
                                v = v.strip().replace(']', '')  # Remove closing bracket if present
                                current_point_values.append(float(v))
                        else:
                            values = line.split(",")
                            for val in values:
                                _, v = val.split("=")
                                v = v.strip().replace(']', '')  # Remove closing bracket if present
                                current_point_values.append(float(v))
                if current_point_name:
                    coordinates_dict[current_point_name] = current_point_values
        except FileNotFoundError:
            print(f"File {file_path} not found.")
        except ValueError as ve:
            print(f"Value error: {ve}")
        except Exception as e:
            print(f"An error occurred: {e}")

        return coordinates_dict

    def setup(self):
        # Initialize the robot with the provided IP
        self.robot = NiryoRobot(self.ip)
        self.robot.calibrate_auto()
        
        # Read coordinates from the file
        coordinates = self.read_coordinates_from_file("points_coordinates.txt")
        
        # Prepare xmllogfile
        self.create_xml()

        # Update points with the read coordinates, raise an error if any point is missing
        try:
            self.observe_point = coordinates["observe_point"]
            self.reload_point = coordinates["reload_point"]
            self.safe_pickpoint = coordinates["safe_pickpoint"]
            self.pickpoint = coordinates["pickpoint"]
            self.conveyor_starting_point = coordinates["conveyor_starting_point"]
            # self.build_point = coordinates["build_point"]
            self.arches_points = [[], [], [], []]
            self.arches_points[0] = [0.126, 0.2988, 0.1431, 2.128, 1.48, -2.647]
            self.arches_points[1] = [0.1527, 0.287, 0.1429, 3.007, 1.498, -1.759]
            self.arches_points[2] = [0.124, 0.2907, 0.1509, -3.041, 1.48, -1.5]
            self.arches_points[3] = [0.1486, 0.2877, 0.1536, -2.59, 1.472, -1.045]
        except KeyError as e:
            print(f"Missing key in the coordinates file: {e}")
            self.observe_point = [0.03123160950542081, 0.27519698024843176, -0.3583151276062615, -0.03365492374369117, -1.7319569631126819, 0.007762557529221059]
            self.reload_point = [-1.445037685001463, -0.5943819036312069, 0.24917813447690063, -0.3588588507754471, -1.3131802080199013, -1.4433832678105953]
            self.safe_pickpoint = [-0.9975911153261807, -0.9034308449403469, 0.6582134979742917, -0.4064122551999021, -1.4251608055355534, -1.0460822437482142]
            self.pickpoint = [-1.0462927827738304, -1.0079621045007912, 0.6339743653225944, -0.2913636961084789, -1.392947208989955, -0.9969948585358739]
            self.conveyor_point = [-0.49535516977229266, -0.4171332461156708, -0.3977037181652696, -0.059732597137747145, -0.7640150859568422, -0.4831112945941842]
            self.build_point = [1.198549701141276, -0.5080299935595355, -0.4810257366554789, 0.016966442256534986, -0.6719762386837038, -0.394140408896817]

        # Move the robot joints to initial positions
        self.robot.move_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        # Set the conveyor for the robot
        self.conveyor_id = self.robot.set_conveyor()

    def reconfigure(self):
        '''Redefines the full configuration for using the robot'''
        self.define_observing_point()
        self.define_reload_point()
        self.define_picking_point()
        self.define_conveyor_starting_point()
        self.define_build_starting_point()
        self.define_arches_points()

    def load_pattern(self):
        # List all files in the current directory
        files = os.listdir("./")
        # Loop through the files and print the ones with "chaikmat_pattern.xml" in their name
        for i in range(len(files)):
            if "chaikmat_pattern.xml" in files[i]:
                print(f"[{i}]: {files[i]}\n")
        # Prompt the user to choose a pattern
        fic = input("Choose a pattern: ")
        lec = open(files[int(fic)]).readlines()
        print(lec)
        # Run the selected pattern
        self.run_pattern(files[int(fic)])

    def run_pattern(self, patfile):
        # Read the pattern from the given pattern file
        lines = open(patfile).readlines()
        final_pick_position = []
        for i in range(len(lines)):
            final_position = []
            # If the line has pickpoint data, store it
            if "pickpoint" in lines[i]:
                print(lines[i])
                pick_position = lines[i].replace("</pickpoint>", "").replace("<pickpoint>", "").rstrip("\n").replace("[", "").replace("]", "")
                for elt in pick_position.split(","):
                    final_pick_position.append(elt)
                continue
            else:
                print("reaching point:")
                position = lines[i].rstrip("\n").replace("[", "").replace("]", "")
                print(position)
                print(type(position))
                for elt in position.split(","):
                    final_position.append(elt)
            # Run the conveyor until the sensor detects an object
            self.robot.run_conveyor(self.conveyor_id)
            try:
                print("reading sensor")
                print(self.robot.digital_read(self.sensor))
                while self.robot.digital_read(self.sensor) == PinState.HIGH:
                    self.robot.wait(0.1)
                self.robot.stop_conveyor(self.conveyor_id)
            except:
                # Handle any sensor errors
                print("error sensor")
                self.robot.stop_conveyor(self.conveyor_id)
            # Move the robot to the pick position, grasp the object, and then move to the defined position to release the object
            self.robot.move_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            self.robot.move_pose(final_pick_position)
            self.robot.grasp_with_tool()
            if not asyncio.run(self.check_pressure()):
                print("No object detected in the grip. Grasp failed.")
                self.robot.release_with_tool()
                return
            self.robot.move_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
            self.robot.move_pose(final_position)
            self.robot.release_with_tool()
        # Move the robot back to the initial position and stop the conveyor
        self.robot.move_pose(0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.robot.stop_conveyor(self.conveyor_id)

    def define_pattern(self):
        # Define a new pattern
        name = input("Please give a name for the pattern: ")
        pat = open(name + "chaikmat_pattern.xml", 'w')
        pat.write("<pickpoint>" + str(self.pickpoint) + "</pickpoint>\n")
        # Allow the user to define new stations for the pattern
        step = input("Define a new station: d\nEnd pattern: e\nYour choice: ")
        while(step != "e"):
            if step == "d":
                shape = input("What should be the shape of the element for this station?\nany: a")
                userchoice = input("Please use free motion to place the arm to the next station and press enter")
                station = self.robot.get_pose()
                pat.write(str(station) + "\n")
                step = input("Define a new station: d\nEnd pattern: e\nYour choice: ")
        pat.close()
        self.user_choice()

    def define_picking_point(self):
        # Allow the user to define a new picking point
        userchoice = input("Please use free motion to place the arm to the PREPARING point for picking and press enter")
        self.safe_pickpoint = self.robot.get_pose()
        print("New PREPARING picking point defined at coordinates:" + str(self.pickpoint) + "")
        with open("points_coordinates.txt", "a") as file:
            file.write("safe_pickpoint: " + str(self.safe_pickpoint) + "\n")

        userchoice = input("Please use free motion to place the arm to the picking point and press enter")
        self.pickpoint = self.robot.get_pose()
        print("New picking point defined at coordinates:" + str(self.pickpoint) + "")
        with open("points_coordinates.txt", "a+") as file:
            file.write("pickpoint: " + str(self.pickpoint) + "\n")

    def define_arches_points(self):
        self.arches_points = [[], [], [], []]
        # Allow the user to define a new arch
        userchoice = input("Please use free motion to place the arm to the left base point and press enter")
        self.arches_points[0] = self.robot.get_pose()
        print("New point defined at coordinates:" + str(self.arches_points[0]))
        userchoice = input("Please use free motion to place the arm to the right base point and press enter")
        self.arches_points[1] = self.robot.get_pose()
        print("New point defined at coordinates:" + str(self.arches_points[1]))
        userchoice = input("Please use free motion to place the arm to the first pillar point and press enter")
        self.arches_points[2] = self.robot.get_pose()
        print("New point defined at coordinates:" + str(self.arches_points[2]))
        userchoice = input("Please use free motion to place the arm to the second pillar point and press enter")
        self.arches_points[3] = self.robot.get_pose()
        print("New point defined at coordinates:" + str(self.arches_points[3]))
        with open("points_coordinates.txt", "a+") as file:
            file.write("arches_point: " + str(self.arches_points) + "\n")

    def define_observing_point(self):
        # Allow the user to define a new observing point
        userchoice = input("Please use free motion to place the arm to the OBSERVING point and press enter")
        self.observe_point = self.robot.get_pose()
        print("New observing point defined at coordinates:" + str(self.observe_point) + "")
        with open("points_coordinates.txt", "w") as file:
            file.write("observe_point: " + str(self.observe_point) + "\n")

    def define_reload_point(self):
        # Allow the user to define a new reload point
        userchoice = input("Please use free motion to place the arm to the RELOAD point and press enter")
        self.reload_point = self.robot.get_pose()
        print("New reload point defined at coordinates:" + str(self.reload_point) + "")
        with open("points_coordinates.txt", "a") as file:
            file.write("reload_point: " + str(self.reload_point) + "\n")

    def define_conveyor_starting_point(self):
        # Allow the user to define a new conveyor starting point
        userchoice = input("Please use free motion to place the arm to the CONVEYOR STARTING point and press enter")
        self.conveyor_starting_point = self.robot.get_pose()
        print("New conveyor point defined at coordinates:" + str(self.conveyor_starting_point) + "\n")
        with open("points_coordinates.txt", "a") as file:
            file.write("conveyor_starting_point: " + str(self.conveyor_starting_point) + "\n")

    def define_build_starting_point(self):
        # Allow the user to define a new build starting point
        userchoice = input("Please use free motion to place the arm to the BUILD STARTING point and press enter")
        self.build_point = self.robot.get_pose()
        print("New build starting point defined at coordinates:" + str(self.build_point) + "\n")
        with open("points_coordinates.txt", "a") as file:
            file.write("build_point: " + str(self.build_point) + "\n")

    def new_workspace(self):
        # Allow the user to define a new workspace point
        print("Please define first point as the one with black center, and then proceed clockwise")
        self.wp_name = "modified"
        try:
            self.robot.delete_workspace(self.wp_name)
        except:
            pass
        workspace_stations = {}
        for i in range(4):
            input("Please use free motion to place the arm to the NEXT workspace point and press enter")
            workspace_stations[i] = self.robot.get_pose()
            print("New workspace point defined at coordinates:" + str(self.robot.get_pose()) + "")
        for elt in workspace_stations:
            print(elt)
            print(workspace_stations[elt])
        self.robot.save_workspace_from_robot_poses(self.wp_name, workspace_stations[0], workspace_stations[1], workspace_stations[2], workspace_stations[3])
        # self.robot.save_workspace_from_points()

    def user_choice(self):
        # Present the user with a series of choices
        userchoice = input("default setup: s \ndefine picking point: p \nDefine a new pattern: d \nLoad a pattern: l\nYour choice? ")
        if userchoice == "p":
            self.define_picking_point()
        elif userchoice == "d":
            self.define_pattern()
        elif userchoice == "l":
            self.load_pattern()
        elif userchoice == "s":
            self.setup()
        else:
            print("Unknown command: %s", userchoice)

    def object_chars(self):
        # Move to observe point and detect objects
        print("Moving to observe point to detect objects...")
        self.robot.move_pose(self.observe_point)
        self.xml_update("observe_point", self.observe_point)
        things = self.robot.detect_object(self.wp_name, shape=ObjectShape.ANY, color=ObjectColor.ANY)
        print("Objects detected:", things)
        return things

    def select_object(self):
        # Move to observe point and select object
        print("Moving to observe point to select object...")
        self.robot.move_pose(self.observe_point)
        self.xml_update("observe_point", self.observe_point)
        obj_found, pos_array, shape, color = self.robot.detect_object(self.wp_name, shape=ObjectShape.ANY, color=ObjectColor.ANY)
        print("Shape:", shape)
        print("Color:", color)

    def vpick(self, s=ObjectShape.ANY, c=ObjectColor.ANY):
        print(f"Vision picking object with shape {s} and color {c}...")
        self.robot.vision_pick(self.wp_name, height_offset=-0.005, shape=s, color=c)

    def neutral(self):
        print("Moving to neutral position (observe point)...")
        self.robot.move_pose(self.observe_point)
        self.xml_update("observe_point", self.observe_point)

    def convey_until_detect(self):
        # Update robot state to Active before operation
        self.robot_state_update(self.ned_aas_file, "Active")
        start_time = time.time()

        # Check service availability
        if "Error" in self.service_query(self.ned_aas_file, "PresenceDetectionOnConveyor"):
            print("Service 'PresenceDetectionOnConveyor' not available in Niryo AAS.")
            self.robot_state_update(self.ned_aas_file, "Idle")
            return

        print("Running conveyor until detection...")
        self.robot.run_conveyor(self.conveyor_id)
        detected = 1
        try:
            print("Reading sensor...")
            print(self.robot.digital_read(self.sensor))
            tic = time.perf_counter()
            while self.robot.digital_read(self.sensor) == PinState.HIGH:
                self.robot.wait(0.1)
                toc = time.perf_counter()
                if toc - tic > 10:
                    self.logfile.write("Unsuccessful Picking attempt\n")
                    self.logfile.flush()
                    self.xml_update("ir_detection", 0)
                    detected = 0
                    break
            self.robot.stop_conveyor(self.conveyor_id)
        except:
            # Handle any sensor errors
            print("Error with sensor")
            self.robot.stop_conveyor(self.conveyor_id)
            self.xml_update("ir_detection", 0)
            detected = 0
        if detected == 1:
            self.xml_update("ir_detection", 1)

        # Log success
        end_time = time.time()
        execution_time = end_time - start_time
        self.service_logs("Convey Until Detect", True, execution_time)

        # Update robot state to Idle after operation
        self.robot_state_update(self.ned_aas_file, "Idle")

    def load_piece(self):
        start_time = time.time()
        # Update robot state to Active before operation
        self.robot_state_update(self.ned_aas_file, "Active")
        
        # Check service availability
        if "Error" in self.service_query(self.ned_aas_file, "Pick"):
            print("Service 'Pick' not available in Niryo AAS.")
            self.robot_state_update(self.ned_aas_file, "Idle")
            return

        print("Loading piece...")
        self.robot.move_pose(self.safe_pickpoint)
        self.xml_update("safe_pickpoint", self.safe_pickpoint)
        self.robot.move_pose(self.pickpoint)
        self.xml_update("pickpoint", self.pickpoint)
        self.robot.grasp_with_tool()
        if not asyncio.run(self.check_pressure()):
            print("No object detected in the grip. Grasp failed.")
            self.robot.release_with_tool()
            self.robot_state_update(self.ned_aas_file, "Idle")
            return

        self.robot.move_pose(self.safe_pickpoint)
        self.xml_update("safe_pickpoint", self.safe_pickpoint)
        self.robot.move_pose(self.conveyor_starting_point)
        self.xml_update("safe_pickpoint", self.conveyor_starting_point)
        self.robot.release_with_tool()

        # Log success
        end_time = time.time()
        execution_time = end_time - start_time
        self.service_logs("Load Piece", True, execution_time)

        # Update robot state to Idle after operation
        self.robot_state_update(self.ned_aas_file, "Idle")

    def put_back_piece(self):
        start_time = time.time()
        # Update robot state to Active before operation
        self.robot_state_update(self.ned_aas_file, "Active")

        # Check service availability
        if "Error" in self.service_query(self.ned_aas_file, "Place"):
            print("Service 'Place' not available in Niryo AAS.")
            self.robot_state_update(self.ned_aas_file, "Idle")
            return

        print("Putting back piece...")
        self.robot.move_pose(self.reload_point)
        self.xml_update("reload_point", self.reload_point) 
        self.robot.release_with_tool()

        # Log success
        end_time = time.time()
        execution_time = end_time - start_time
        self.service_logs("Put Back Piece", True, execution_time)

        # Update robot state to Idle after operation
        self.robot_state_update(self.ned_aas_file, "Idle")

    def pick_my_thing(self, shape_exp, color_exp, number):
        if shape_exp == "Square":
            shape_expected = ObjectShape.SQUARE
        else:
            shape_expected = ObjectShape.CIRCLE

        if color_exp == "Red":
            color_expected = ObjectColor.RED
        elif color_exp == "Green":
            color_expected = ObjectColor.GREEN
        else:
            color_expected = ObjectColor.BLUE

        start_time = time.time()

        # Update robot state to Active before operation
        self.robot_state_update(self.ned_aas_file, "Active")

        # Check service availability
        if "Error" in self.service_query(self.ned_aas_file, "ColorAndShapeDetection"):
            print("Service 'ColorAndShapeDetection' not available in Niryo AAS.")
            self.robot_state_update(self.ned_aas_file, "Idle")
            return 0

        # Moving to observation point
        print("Moving to observation point...")
        self.robot.move_pose(self.observe_point)
        self.xml_update("observe_point", self.observe_point)
        obj_found, pos_array, shape, color = self.robot.detect_object(self.wp_name,
            shape=shape_expected,
            color=color_expected)
        print("Object found:", obj_found)
        self.vpick()
        if shape == shape_expected and color == color_expected:
            print("We have what we want")
            self.neutral()
            self.robot.move_pose(self.arches_points[number])
            self.xml_update("Build_point_" + str(number), self.arches_points[number])
            self.robot.release_with_tool()

            # Log success
            end_time = time.time()
            execution_time = end_time - start_time
            self.service_logs("Pick and Place", True, execution_time)

            # Update robot state to Idle after operation
            self.robot_state_update(self.ned_aas_file, "Idle")
            return 1
        else:
            print("Not the good piece")
            self.put_back_piece()
            
            # Log failure
            self.service_logs("Pick and Place", False, 0.0)

            # Update robot state to Idle after operation
            self.robot_state_update(self.ned_aas_file, "Idle")
            return 0

    async def check_pressure(self):
        pressure = await get_pressure()
        print(f"Pressure: {pressure}")
        threshold_value = 0.5  # Replace with an appropriate threshold for object detection
        return pressure > threshold_value

###################################################################################################################################################################
    """ DECLARATIONS OF AAS SERVICE COMPOSITION INTERFACES FUNCTIONS """
###################################################################################################################################################################


    def service_query(self, file_path, service_name):
        """
        The Service Query Interface facilitates the retrieval of data from the AAS.
        It allows clients to query the Services submodel and obtain the necessary information,
        for instance, the availability and parameters of services, before performing tasks.

        For example, before the execution of a pick operation by the robot, the Control App
        will query the AAS to check if the concerned service exists in its registry and to
        obtain information about its input, output, driver function, and effector.
        If the service is not found, an error will be returned. The process might continue
        executing in case it was a mistake on the AAS side.
        """
        try:
            # Parse the XML file with namespace handling
            tree = ET.parse(file_path)
            root = tree.getroot()
            
            # Define the namespaces
            ns = {'aas': 'https://admin-shell.io/aas/3/0'}
            
            # Debug: Print root element
            print(f"Root element: {root.tag}")
            
            # Find the 'Services' element using the namespace
            services = root.find(".//aas:submodelElementCollection[aas:idShort='Services']/aas:value", ns)
            
            # Debug: Print if services element is found
            if services is not None:
                print("Found 'Services' element")
            else:
                print("Did not find 'Services' element")
                return "Error: 'Services' element not found."
            
            # Iterate through each service in the services list
            for service in services.findall("aas:submodelElementCollection", ns):
                id_short = service.find("aas:idShort", ns)
                # Debug: Print each service idShort
                if id_short is not None:
                    print(f"Found service with idShort: {id_short.text}")
                    
                if id_short is not None and id_short.text == service_name:
                    # Find the DriverFunction property within the service
                    driver_function = service.find(".//aas:property[aas:idShort='DriverFunction']/aas:value", ns)
                    input_param = service.find(".//aas:property[aas:idShort='Input']/aas:value", ns)
                    output_param = service.find(".//aas:property[aas:idShort='Output']/aas:value", ns)
                    effector = service.find(".//aas:property[aas:idShort='Effector']/aas:value", ns)
                    
                    if driver_function is not None:
                        driver_function_text = driver_function.text
                    else:
                        driver_function_text = "Not specified"
                    
                    if input_param is not None:
                        input_param_text = input_param.text
                    else:
                        input_param_text = "Not specified"
                    
                    if output_param is not None:
                        output_param_text = output_param.text
                    else:
                        output_param_text = "Not specified"
                    
                    if effector is not None:
                        effector_text = effector.text
                    else:
                        effector_text = "Not specified"
                    
                    result = (
                        f"The '{service_name}' service is available with the following details:\n"
                        f"Driver Function: {driver_function_text}\n"
                        f"Input: {input_param_text}\n"
                        f"Output: {output_param_text}\n"
                        f"Effector: {effector_text}"
                    )
                    print(result)
                    return result
            
            error_message = f"Error: The '{service_name}' service is not available."
            print(error_message)
            return error_message
        
        except ET.ParseError as e:
            error_message = f"Error parsing XML file: {e}"
            print(error_message)
            return error_message
        except Exception as e:
            error_message = f"An error occurred: {e}"
            print(error_message)
            return error_message




############################################################################################################################################

    def service_logs(self, service_name, success, execution_time):
        """
        The Service Logs Interface is responsible for logging and monitoring the activities
        and performance of various services and recording them in a logfile. This is achieved
        through the function 'service_logs()'.
        
        When a service is called, its status and execution time are appended to the logfile
        'service metrics.txt' for analysis of our production chain performance and improvement.
        """
        # Determine the status of the service based on the success parameter
        status = "SUCCESS" if success else "FAILURE"
        
        # Log performance metrics to a text file
        with open(self.metrics_file, 'a') as f:
            f.write(f"Service: {service_name}, Status: {status}, Execution Time: {execution_time:.2f} seconds\n")
        
        # Print log entry (optional, for console output)
        print(f"Service '{service_name}' executed with {status}. Execution time: {execution_time:.2f} seconds.")


############################################################################################################################################
   
    
    def robot_state_update(file_path, state):
        """
        The Robot State Update Interface is designed to manage the operational states of robotic assets
        within the AAS (Asset Administration Shell). This interface allows for dynamic modifications to
        represent each robot's current state, such as Active, Inactive, or Maintenance. The function
        'robot_state_update()' updates the operational state of the robot in the AAS XML file.
        """
        try:
            # Parse the XML file with namespace handling
            tree = ET.parse(file_path)
            root = tree.getroot()
            ns = {'aas': 'https://admin-shell.io/aas/3/0'}
            
            # Find the 'OperationalStates' element using the namespace
            operational_states = root.find(".//aas:submodelElementCollection[aas:idShort='OperationalStates']/aas:value", ns)
            if operational_states is None:
                print("Did not find 'OperationalStates' element")
                return "Error: 'OperationalStates' element not found."

            # Valid states list
            valid_states = ["Idle", "Active", "Error"]

            if state not in valid_states:
                print(f"Invalid state: {state}. Valid states are {valid_states}.")
                return f"Error: Invalid state '{state}'. Valid states are {valid_states}."

            # Update the state
            state_updated = False
            for state_property in operational_states.findall("aas:property", ns):
                id_short = state_property.find("aas:idShort", ns)
                if id_short is not None:
                    state_value = state_property.find("aas:value", ns)
                    if id_short.text == state:
                        state_value.text = "true"
                        state_updated = True
                    else:
                        if id_short.text in valid_states:
                            state_value.text = "false"

            if state_updated:
                # Save the updated XML file
                tree.write(file_path)
                print(f"The robot state has been updated to '{state}'.")
                return f"The robot state has been updated to '{state}'."
            else:
                print(f"The state '{state}' was not found in 'OperationalStates'.")
                return f"Error: The state '{state}' was not found in 'OperationalStates'."
        except ET.ParseError as e:
            error_message = f"Error parsing XML file: {e}"
            print(error_message)
            return error_message
        except Exception as e:
            error_message = f"An error occurred: {e}"
            print(error_message)
            return error_message



###########################################################################################################################################


    def configure_service(file_path, service_name, input_value, output_value, driver_function, effector):
        """
        The Dynamic Service Configuration Interface enables updates to services within the AAS,
        allowing users to add, remove, and modify service configurations dynamically.
        It is crucial for maintaining an adaptive and responsive production environment.
        
        This function dynamically configures the specified service in the AAS XML file.
        
        Parameters:
            file_path (str): The path to the AAS XML file.
            service_name (str): The name of the service to be configured.
            input_value (str): The input value to be set for the service.
            output_value (str): The output value to be set for the service.
            driver_function (str): The driver function to be set for the service.
            effector (str): The effector value to be set for the service.
        """
        try:
            tree = ET.parse(file_path)
            root = tree.getroot()
            ns = {'aas': 'https://admin-shell.io/aas/3/0'}
            
            services = root.find(".//aas:submodelElementCollection[aas:idShort='Services']/aas:value", ns)
            if services is None:
                print("Did not find 'Services' element")
                return

            for service in services.findall("aas:submodelElementCollection", ns):
                id_short = service.find("aas:idShort", ns)
                if id_short is not None and id_short.text == service_name:
                    # Update service properties
                    service.find(".//aas:property[aas:idShort='Input']/aas:value", ns).text = input_value
                    service.find(".//aas:property[aas:idShort='Output']/aas:value", ns).text = output_value
                    service.find(".//aas:property[aas:idShort='DriverFunction']/aas:value", ns).text = driver_function
                    service.find(".//aas:property[aas:idShort='Effector']/aas:value", ns).text = effector

                    # Save the updated XML file
                    tree.write(file_path)
                    print(f"The '{service_name}' service has been configured.")
                    return

            print(f"The '{service_name}' service is not available.")
        except ET.ParseError as e:
            print(f"Error parsing XML file: {e}")
        except Exception as e:
            print(f"An error occurred: {e}")


###########################################################################################################################################


    def add_service(file_path, service_name, input_value, output_value, driver_function, effector):
        """
        The Dynamic Service Configuration Interface enables the dynamic addition of new services,
        integrating with existing systems.

        This function adds a new service to the AAS XML file.

        Parameters:
            file_path (str): The path to the AAS XML file.
            service_name (str): The name of the new service to be added.
            input_value (str): The input value to be set for the new service.
            output_value (str): The output value to be set for the new service.
            driver_function (str): The driver function to be set for the new service.
            effector (str): The effector value to be set for the new service.
        """
        try:
            tree = ET.parse(file_path)
            root = tree.getroot()
            ns = {'aas': 'https://admin-shell.io/aas/3/0'}

            services = root.find(".//aas:submodelElementCollection[aas:idShort='Services']/aas:value", ns)
            if services is None:
                print("Did not find 'Services' element")
                return

            new_service = ET.Element("aas:submodelElementCollection", ns)
            id_short = ET.SubElement(new_service, "aas:idShort", ns)
            id_short.text = service_name

            input_prop = ET.SubElement(new_service, "aas:property", ns)
            input_id = ET.SubElement(input_prop, "aas:idShort", ns)
            input_id.text = "Input"
            input_val = ET.SubElement(input_prop, "aas:value", ns)
            input_val.text = input_value

            output_prop = ET.SubElement(new_service, "aas:property", ns)
            output_id = ET.SubElement(output_prop, "aas:idShort", ns)
            output_id.text = "Output"
            output_val = ET.SubElement(output_prop, "aas:value", ns)
            output_val.text = output_value

            driver_prop = ET.SubElement(new_service, "aas:property", ns)
            driver_id = ET.SubElement(driver_prop, "aas:idShort", ns)
            driver_id.text = "DriverFunction"
            driver_val = ET.SubElement(driver_prop, "aas:value", ns)
            driver_val.text = driver_function

            effector_prop = ET.SubElement(new_service, "aas:property", ns)
            effector_id = ET.SubElement(effector_prop, "aas:idShort", ns)
            effector_id.text = "Effector"
            effector_val = ET.SubElement(effector_prop, "aas:value", ns)
            effector_val.text = effector

            services.append(new_service)

            # Save the updated XML file
            tree.write(file_path)
            print(f"The '{service_name}' service has been added.")
        except ET.ParseError as e:
            print(f"Error parsing XML file: {e}")
        except Exception as e:
            print(f"An error occurred: {e}")
            

###########################################################################################################################################


    def remove_service(file_path, service_name):
        """
        The Dynamic Service Configuration Interface allows for the removal of unnecessary services
        to optimize performance and resource utilization.

        This function removes a specified service from the AAS XML file.

        Parameters:
            file_path (str): The path to the AAS XML file.
            service_name (str): The name of the service to be removed.
        """
        try:
            tree = ET.parse(file_path)
            root = tree.getroot()
            ns = {'aas': 'https://admin-shell.io/aas/3/0'}

            services = root.find(".//aas:submodelElementCollection[aas:idShort='Services']/aas:value", ns)
            if services is None:
                print("Did not find 'Services' element")
                return

            for service in services.findall("aas:submodelElementCollection", ns):
                id_short = service.find("aas:idShort", ns)
                if id_short is not None and id_short.text == service_name:
                    services.remove(service)
                    tree.write(file_path)
                    print(f"The '{service_name}' service has been removed.")
                    return

            print(f"The '{service_name}' service is not available.")
        except ET.ParseError as e:
            print(f"Error parsing XML file: {e}")
        except Exception as e:
            print(f"An error occurred: {e}")


#####################################################################################################################################################
        
