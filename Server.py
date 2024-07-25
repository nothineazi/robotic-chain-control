import asyncio
from asyncua import ua, Server
import logging
import os

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger('asyncua')

OPC_UA_URL = "opc.tcp://172.20.10.3:4840/opcua/"
NAMESPACE = "mynamespace"
PRESSURE_VALUE = 101.3
SERVER_FILE_DIRECTORY = os.path.expanduser("~/Desktop/")
AASX_FILES = {
    "NiryoNed2AAS.aasx": os.path.join(SERVER_FILE_DIRECTORY, "NiryoNed2AAS.aasx"),
    "WlkataMirobotAAS.aasx": os.path.join(SERVER_FILE_DIRECTORY, "WlkataMirobotAAS.aasx")
}

async def main():
    # Create server instance
    server = Server()
    await server.init()
    server.set_endpoint(OPC_UA_URL)
    uri = NAMESPACE
    idx = await server.register_namespace(uri)

    # Get the Objects node, this is where we should put our nodes
    objects = server.nodes.objects

    # Add a new object to the address space
    vPLC = await objects.add_object(idx, "vPLC")

    # Add a variable to the vPLC object
    press_var = await vPLC.add_variable(idx, "pression", PRESSURE_VALUE)
    await press_var.set_writable()

    # Add file nodes to the vPLC object
    for file_name, file_path in AASX_FILES.items():
        file_content = read_file(file_path)
        file_node = await vPLC.add_variable(idx, file_name, ua.Variant(file_content, ua.VariantType.ByteString))
        await file_node.set_writable()

    # Start the server
    async with server:
        logger.info(f"Server started at {OPC_UA_URL}")
        while True:
            await asyncio.sleep(1)

def read_file(file_path):
    """
    Reads the file content and returns it as bytes.
    """
    with open(file_path, 'rb') as f:
        return f.read()

if __name__ == "__main__":
    asyncio.run(main())
