import asyncio
import logging
from asyncua import Client, ua
import aiofiles
import os

logger = logging.getLogger('asyncua')
logging.disable(logging.WARNING)

OPC_UA_URL = "opc.tcp://172.20.10.3:4840/opcua/"
NAMESPACE = "mynamespace"
DATA_VARIABLES = ["pression"]
AASX_FILES = ["NiryoNed2AAS.aasx", "WlkataMirobotAAS.aasx"]
CLIENT_FILE_PATH = os.path.expanduser("~/Runchain_Services/")

async def get_pressure():
    """
    Récupère la valeur de la pression à partir du serveur OPC-UA.
    """
    async with Client(url=OPC_UA_URL) as client:
        namespace_idx = await client.get_namespace_index(NAMESPACE)
        myvar = await client.nodes.root.get_child([
            "0:Objects", 
            "{}:vPLC".format(namespace_idx), 
            "{}:pression".format(namespace_idx)
        ])
        pressure_value = await myvar.get_value()
        return pressure_value

async def get_aasx_file(file_name):
    """
    Télécharge un fichier AASX à partir du serveur OPC-UA.
    """
    async with Client(url=OPC_UA_URL) as client:
        namespace_idx = await client.get_namespace_index(NAMESPACE)
        aasx_node = await client.nodes.root.get_child([
            "0:Objects",
            "{}:vPLC".format(namespace_idx),
            "{}:{}".format(namespace_idx, file_name)
        ])
        file_content = await aasx_node.get_value()
        
        async with aiofiles.open(os.path.join(CLIENT_FILE_PATH, file_name), 'wb') as f:
            await f.write(file_content)

async def main():
    await get_pressure()

    for file_name in AASX_FILES:
        await get_aasx_file(file_name)
    print("Files imported successfully.")

if __name__ == "__main__":
    asyncio.run(main())
