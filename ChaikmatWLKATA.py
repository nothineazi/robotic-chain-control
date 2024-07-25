from math import pi
from mirobot import *
from wlkata_mirobot import WlkataMirobot
import time
import schedule
import xml.etree.ElementTree as ET
from datetime import datetime 

# Define the AAS file path for Wlkata
wl_aas_file = "~/Runchain_Services/WlkataMirobotAAS.aas.xml"


class ChaikWLKATA:
    def __init__(self):
        self.arm = WlkataMirobot()
        self.arm.home()
        print(self.arm.get_status())
        self.message = self.arm.get_status()

        # Initialize metrics storage
        self.metrics_file = "service_metrics.txt"



    def pick(self, color, shape):
        start_time = time.time()
        
        # Update robot state to Active before operation
        self.robot_state_update(wl_aas_file, "Active")
        
        # Check service availability
        if "Error" in self.service_query(wl_aas_file, "Pick"):
            print("Service 'Pick' not available in Wlkata AAS.")
            self.robot_state_update(wl_aas_file, "Idle")
            return False
        
        try:
            # Perform picking based on color and shape
            if color == "blue":
                if shape == "square":
                    target_angles = {1: -19.8, 2: 60.0, 3: -10.0, 4: 0.0, 5: -59.8, 6: 0.0}
                    self.arm.set_joint_angle(target_angles)
                    self.arm.pump_suction()
                elif shape == "circle":
                    target_angles = {1: -15.0, 2: 70.0, 3: -25.0, 4: 10.0, 5: -39.7, 6: -20.0}
                    self.arm.set_joint_angle(target_angles)
                    self.arm.pump_suction()
            elif color == "red":
                if shape == "square":
                    target_angles = {1: -39.9, 2: 60.0, 3: 10.0, 4: 5.0, 5: -44.7, 6: 0.0}
                    self.arm.set_joint_angle(target_angles)
                    self.arm.pump_suction()
                elif shape == "circle":
                    target_angles = {1: -28.4, 2: 70.0, 3: -20.5, 4: 2.4, 5: -30.7, 6: 7.0}
                    self.arm.set_joint_angle(target_angles)
                    self.arm.pump_suction()
            elif color == "green":
                if shape == "square":
                    target_angles = {1: -15, 2: 50.0, 3: 25.0, 4: -5.0, 5: -48.7, 6: -5.0}
                    self.arm.set_joint_angle(target_angles)
                    self.arm.pump_suction()
                elif shape == "circle":
                    target_angles = {1: -29.9, 2: 45.0, 3: 25.0, 4: -5.0, 5: -79.7, 6: 0.0}
                    self.arm.set_joint_angle(target_angles)
                    self.arm.pump_suction()
                    
            # Log success
            end_time = time.time()
            execution_time = end_time - start_time
            self.service_logs("Pick", True, execution_time)
            self.robot_state_update(wl_aas_file, "Idle")
            return True
        
        except Exception as e:
            print(f"Error picking object: {e}")
            # Log failure
            self.service_logs("Pick", False, 0.0)
            self.robot_state_update(wl_aas_file, "Idle")
            return False

    def release_to_Ned(self):
        start_time = time.time()
        
        # Update robot state to Active before operation
        self.robot_state_update(wl_aas_file, "Active")

        # Check service availability
        if "Error" in self.service_query(wl_aas_file, "Place"):
            print("Service 'Place' not available in Wlkata AAS.")
            self.robot_state_update(wl_aas_file, "Idle")
            return False
        
        try:
            # Release operation to Ned ramp
            target_angles = {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0, 5: 0.0, 6: 0.0}
            self.arm.set_joint_angle(target_angles)

            target_angles = {1: -75.0, 2: 20.0, 3: -15.0, 4: 5.0, 5: -5.0, 6: 0.0}
            self.arm.set_joint_angle(target_angles)
            self.arm.pump_off()

            target_angles = {1: 0.0, 2: 0.0, 3: 0.0, 4: 0.0, 5: 0.0, 6: 0.0}
            self.arm.set_joint_angle(target_angles)
            
            # Log success
            end_time = time.time()
            execution_time = end_time - start_time
            self.service_logs("Release", True, execution_time)
            self.robot_state_update(wl_aas_file, "Idle")
            return True
        
        except Exception as e:
            print(f"Error releasing object: {e}")
            # Log failure
            self.service_logs("Release", False, 0.0)
            self.robot_state_update(wl_aas_file, "Idle")
            return False

    def move_conveyor(self, position):
        start_time = time.time()

        # Update robot state to Active before operation
        self.robot_state_update(wl_aas_file, "Active")
        
        # Check service availability
        if "Error" in self.service_query(wl_aas_file, "Move"):
            print("Service 'Move' not available in Wlkata AAS.")
            self.robot_state_update(wl_aas_file, "Idle")
            return False

        try:
            self.arm.set_conveyor_pos(position, is_relative=True)
            # Log success
            end_time = time.time()
            execution_time = end_time - start_time
            self.service_logs("Move Conveyor", True, execution_time)
            self.robot_state_update(wl_aas_file, "Idle")
            return True
        except Exception as e:
            print(f"Error moving conveyor: {e}")
            # Log failure
            self.service_logs("Move Conveyor", False, 0.0)
            self.robot_state_update(wl_aas_file, "Idle")
            return False


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
    
