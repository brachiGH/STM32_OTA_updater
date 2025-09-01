import uvicorn
from fastapi import FastAPI, HTTPException
from fastapi import Response
import random
import os

def addCRC(data: bytes):
     crc: int = 0
     for i in range(0, len(data), 4):
                 chunk = data[i:i+4]
                 # Pad the last chunk if it's smaller than 4 bytes
                 if len(chunk) < 4:
                       chunk = chunk.ljust(4, b'\x00')
                 val = int.from_bytes(chunk, byteorder='little')
                 crc ^= val
     return crc.to_bytes(4, byteorder='little') + data

def lastesVersion(deviceModelID: int):
     path = f"./deviceModels/{deviceModelID}"
     try:
          # List all entries in the directory
          entries = os.listdir(path)
          # Filter for directories and convert their names to integers
          version_numbers = [int(entry) for entry in entries if os.path.isdir(os.path.join(path, entry)) and entry.isdigit()]
          # Return the highest version number, or None if no version folders exist
          return max(version_numbers) if version_numbers else None
     except FileNotFoundError:
          # The directory for the device model does not exist
          return None


# --- Application Setup ---
app = FastAPI(
     title="Firmware Updater Server",
     description="A server that is a part of a demonstration of OTA update.",
     version="1.0.0",
)

# --- API Endpoints ---
@app.get("/")
async def read_root():
     return {"message": "Welcome to update server!"}

@app.get("/device/{deviceModelID}/latest")
async def getVersionID(deviceModelID: int):
     version = lastesVersion(deviceModelID)
     if (version == None):
          return HTTPException(status_code=400, detail="Invalid device ID")
     return Response(content=addCRC(version.to_bytes(4, byteorder='little')), media_type="application/octet-stream")

@app.get("/update/{deviceModelID}/{versionID}/{cursorKb}")
async def getVersionID(deviceModelID: int, versionID: int, cursorKb: int):
     file_path = f"./deviceModels/{deviceModelID}/{versionID}/application.bin"
     offset = cursorKb * 1020
     chunk_size = 1020

     try:
          with open(file_path, "rb") as f:
               f.seek(offset)
               chunk = f.read(chunk_size)
               if len(chunk) < chunk_size:
                    chunk = b'\x00\x00\x00\x00' + chunk
                    chunk = chunk.ljust(chunk_size + 4, b'\x00')
               else:
                    chunk = b'\xff\xff\xff\xff' + chunk
               return Response(content=addCRC(chunk), media_type="application/octet-stream")
     except FileNotFoundError:
          raise HTTPException(status_code=404, detail="Firmware file not found")


# --- Server Execution ---
if __name__ == "__main__":
     # Uvicorn is a lightning-fast ASGI server, built on uvloop and httptools.
     # We configure it to use the SSL key and certificate files.
     uvicorn.run(
          app,
          host="0.0.0.0",
          port=443,
          ssl_keyfile="./cert/server.key",
          ssl_certfile="./cert/server.crt",
          reload=True,
          log_level="info",
     )