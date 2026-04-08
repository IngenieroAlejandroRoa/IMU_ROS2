import asyncio
from bleak import BleakClient

ADDRESS = "C7:B5:46:C9:36:9D"

async def main():
    async with BleakClient(ADDRESS) as client:
        print(f"Conectado: {client.is_connected}\n")

        for service in client.services:
            for char in service.characteristics:
                props = ",".join(char.properties)
                if "notify" in char.properties or "read" in char.properties:
                    print(f"SERVICIO: {service.uuid}")
                    print(f"CHAR:     {char.uuid}")
                    print(f"PROPS:    {props}")
                    print("-" * 50)

asyncio.run(main())
