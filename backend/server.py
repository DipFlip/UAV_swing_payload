from __future__ import annotations

"""FastAPI + WebSocket server for the drone simulation."""

import asyncio
import json
import logging
import traceback
import time

logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import FileResponse
from pathlib import Path

from .physics import Params
from .simulation import Simulation

app = FastAPI()

# Serve frontend static files
frontend_dir = Path(__file__).parent.parent / "frontend"
app.mount("/static", StaticFiles(directory=str(frontend_dir)), name="static")


@app.get("/")
async def root():
    return FileResponse(str(frontend_dir / "index.html"))


@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()

    sim_lqr = Simulation(controller_type="lqr")
    sim_pid = Simulation(controller_type="pid")

    # Send at 25 Hz (40ms), run 8 substeps per frame = 4x realtime
    send_interval = 0.04
    substeps = 8

    # Use a separate task for receiving messages
    pending_messages: list[dict] = []
    receive_closed = False

    async def receive_loop():
        nonlocal receive_closed
        try:
            while True:
                data = await websocket.receive_text()
                pending_messages.append(json.loads(data))
        except Exception:
            receive_closed = True

    recv_task = asyncio.create_task(receive_loop())

    try:
        while not receive_closed:
            t0 = time.monotonic()

            # Process queued messages
            while pending_messages:
                msg = pending_messages.pop(0)
                msg_type = msg.get("type")
                if msg_type == "set_goal":
                    x, y, z = float(msg["x"]), float(msg["y"]), float(msg["z"])
                    sim_lqr.set_goal(x, y, z)
                    sim_pid.set_goal(x, y, z)
                elif msg_type == "set_aggression":
                    aggr = float(msg["value"])
                    sim_lqr.set_aggression(aggr)
                    sim_pid.set_aggression(aggr)
                elif msg_type == "reset":
                    sim_lqr.reset()
                    sim_pid.reset()

            # Step simulations
            for _ in range(substeps):
                state_lqr = sim_lqr.step()
                state_pid = sim_pid.step()

            # Send compact state (strip inner "type" keys to save bytes)
            payload = json.dumps({
                "t": "d",
                "l": state_lqr,
                "p": state_pid,
            }, separators=(',', ':'))
            await websocket.send_text(payload)

            # Sleep for remaining time in frame
            elapsed = time.monotonic() - t0
            sleep_time = send_interval - elapsed
            if sleep_time > 0:
                await asyncio.sleep(sleep_time)

    except WebSocketDisconnect:
        pass
    except Exception as e:
        logger.error(f"WebSocket error: {e}\n{traceback.format_exc()}")
    finally:
        recv_task.cancel()
