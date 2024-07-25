
# CHAIKMAT 4.0 Experimental Robotic Chain Control

# Instructions for Running the Robotic Chain and Using Interfaces

## Introduction
This document provides detailed instructions on how to set up, run, and use the robotic chain control system, which includes Niryo Ned 2 and Wlkata Mirobot robots. It also explains how to utilize the various interfaces provided in the system.

## Prerequisites
Before you begin, ensure that you have the following:
- Python 3.6 or later installed on your PC.
- Niryo One Robot and Wlkata Mirobot set up and connected.
- OPC UA Server running and accessible.
- Necessary Python libraries installed (see `requirements.txt`).

## Setting Up

### Step 1: Clone the Repository
Clone the repository containing the control scripts to your local machine:
```sh
git clone https://github.com/yourusername/robotic-chain-control.git
cd robotic-chain-control
```

### Step 2: Install Dependencies
Install the required Python libraries by running:
```sh
pip install -r requirements.txt
```

### Step 3: Configuration
Ensure the AAS (Asset Administration Shell) file paths are correctly set in the control scripts:
- For Niryo:
  ```python
  ned_aas_file = os.path.expanduser("~/Runchain_Services/NiryoNed2AAS.aas.xml")
  ```
- For Wlkata:
  ```python
  wl_aas_file = os.path.expanduser("~/Runchain_Services/WlkataMirobotAAS.aas.xml")
  ```

Verify that the OPC UA server URL and namespace are correctly configured in `client.py`:
```python
OPC_UA_URL = "opc.tcp://<server-ip>:4840/opcua/"
NAMESPACE = "<namespace>"
```

## Running the Control Script

### Step 1: Start the Control Script
Run the main control script:
```sh
python Demonstrateur.py
```
This script will guide you through various prompts to perform different operations such as defining points, loading patterns, and running patterns.

### Step 2: Example Operations

#### Setting Up the Robots
Initialize and calibrate the robots:
```python
chaikmat_ned.setup()
```

#### Running a Predefined Pattern
Load and run a predefined pattern:
```python
chaikmat_ned.load_pattern()
```

#### Pick and Place Operation
Perform a pick and place operation:
```python
chaikmat_ned.pick_my_thing("Square", "Red", 1)
```

## Using the Interfaces

### Service Query Interface
This interface allows you to query the Services submodel to obtain information about service availability and parameters before performing tasks. For example, to check if a pick service is available:
```python
result = chaikmat_ned.service_query(ned_aas_file, "Pick")
print(result)
```

### Service Logs Interface
This interface logs and monitors the activities and performance of various services. The logs are stored in a logfile for analysis:
```python
chaikmat_ned.service_logs("Pick", True, 3.5)
```

### Robot State Update Interface
This interface manages the operational states of robotic assets within the AAS, such as Active, Inactive, or Maintenance:
```python
chaikmat_ned.robot_state_update(ned_aas_file, "Active")
```

### Dynamic Service Configuration Interface
This interface enables dynamic configuration of services within the AAS, allowing users to add, remove, and modify service configurations:
```python
chaikmat_ned.configure_service(ned_aas_file, "Pick", "input_value", "output_value", "driver_function", "effector")
```

## Monitoring and Logging
The system generates logs for each operation, which are saved in a log file with a timestamp in the filename. These logs provide detailed information about the operations performed and any errors encountered. Ensure to check the log files regularly for monitoring and troubleshooting purposes.

## Troubleshooting
- Ensure all dependencies are installed and the OPC UA server is running.
- Check the configurations in `client.py` and the control scripts.
- Refer to the log files for detailed error messages.

## Ownership
Laboratoire Genie de Production (LGP) - Universite de Technologie Tarbes Occitanie Pyrenees (UTTOP) / Ecole Nationale d'Ingenieurs de Tarbes (ENIT) 
Address: 47, avenue d'Azereix - BP 1629 - 65016 Tarbes CEDEX
Contact: +33 (0)5 62 44 27 00

