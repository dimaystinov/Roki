"""Small reference client, suitable as transport foundation for a Qt adapter."""

import argparse
import asyncio
import ipaddress
import json

from roki_ng.wire import Fault
from roki_ng.wire import UDP_LIMIT
from roki_ng.wire import envelope
from roki_ng.wire import pack
from roki_ng.wire import unpack
from roki_ng.wire import normalize_body
from roki_ng.wire import udp_socket


class Client:
    def __init__(self, host, port=8093, *, local_ip="0.0.0.0"):
        self.address = (str(ipaddress.IPv4Address(host)), port)
        self.sock = udp_socket()
        self.sock.bind((str(ipaddress.IPv4Address(local_ip)), 0))
        self.connected = False
        self.operator_ip = None
        self.last_tx = 0.
        self.last_rx = 0.
        self.id = 0
        self.sequence = 0
        self.pending = {}
        self.events = asyncio.Queue(maxsize=128)
        self.tasks = []

    async def connect(self):
        self.tasks.append(asyncio.create_task(self._read()))
        welcome = await self._exchange("hello", "hello", {})
        if "error" in welcome["body"]:
            error = welcome["body"]["error"]
            raise Fault(error["code"], error["message"])
        self.connected = True
        self.operator_ip = welcome["body"]["operator_ip"]
        self.tasks.append(asyncio.create_task(self._heartbeat()))
        return welcome["body"]

    async def _read(self):
        while True:
            raw, address = await asyncio.get_running_loop().sock_recvfrom(self.sock, UDP_LIMIT + 1)
            if address != self.address:
                continue
            try:
                msg = normalize_body(unpack(raw))
            except Fault:
                continue
            self.last_rx = asyncio.get_running_loop().time()
            future = self.pending.get(msg.get("id"))
            if msg.get("kind") in ("response", "welcome") and future is not None:
                if not future.done():
                    future.set_result(msg)
            elif msg.get("kind") in ("event", "sample"):
                if self.events.full():
                    self.events.get_nowait()
                self.events.put_nowait(msg)

    async def _exchange(self, kind, op, body):
        # No await while assigning IDs. A slow camera request must not block heartbeat.
        self.id += 1
        ident = self.id
        future = asyncio.get_running_loop().create_future()
        self.pending[ident] = future
        data = pack(envelope(kind, op, body, id=ident))
        timeout = 5.0 if op in ("camera.start", "camera.stop", "control.acquire") else 0.25
        try:
            for _ in range(5):
                await asyncio.get_running_loop().sock_sendto(self.sock, data, self.address)
                self.last_tx = asyncio.get_running_loop().time()
                try:
                    return await asyncio.wait_for(asyncio.shield(future), timeout)
                except TimeoutError:
                    pass
            raise Fault("timeout", op)
        finally:
            self.pending.pop(ident, None)
            if not future.done():
                future.cancel()

    async def request(self, op, body=None):
        body = dict(body or {})
        reply = await self._exchange("request", op, body)
        if "error" in reply["body"]:
            error = reply["body"]["error"]
            raise Fault(error["code"], error["message"], error.get("retryable", False))
        result = reply["body"]["result"]
        return result

    async def drive(self, x=0, y=0, yaw=0, speed=0.5, crouch="off", heading_hold=False):
        self.sequence += 1
        data = pack(envelope("sample", "motion.drive", {
            "x": x, "y": y, "yaw": yaw,
            "speed": speed, "crouch": crouch, "heading_hold": heading_hold,
        }, sequence=self.sequence))
        await asyncio.get_running_loop().sock_sendto(self.sock, data, self.address)
        self.last_tx = asyncio.get_running_loop().time()

    async def _heartbeat(self):
        while True:
            await asyncio.sleep(0.5)
            now = asyncio.get_running_loop().time()
            if now - self.last_tx < .5 and now - self.last_rx < .5:
                continue
            try:
                await self.request("session.heartbeat")
            except Fault as exc:
                if self.events.full():
                    self.events.get_nowait()
                self.events.put_nowait({"kind": "event", "op": "client.connection_error", "body": exc.as_dict()})

    async def close(self):
        if self.connected:
            try:
                await self.request("session.close")
            except Fault:
                pass
        for task in self.tasks:
            task.cancel()
        await asyncio.gather(*self.tasks, return_exceptions=True)
        self.sock.close()


async def cli(args):
    client = Client(args.robot, args.port)
    event_task = None
    try:
        print(json.dumps(await client.connect(), ensure_ascii=False))
        if args.control:
            await client.request("control.acquire")
            await client.request("mode.set", {"mode": "MANUAL"})
        await client.request("log.subscribe")

        async def events():
            while True:
                print(json.dumps(await client.events.get(), ensure_ascii=False), flush=True)

        event_task = asyncio.create_task(events())
        print('Enter: operation {JSON arguments}; e.g. motion.pose {"name":"base_stand"}; quit to exit')
        while True:
            try:
                line = await asyncio.to_thread(input, "> ")
            except EOFError:
                break
            if line.strip() in ("quit", "exit"):
                break
            if not line.strip():
                continue
            op, _, body = line.partition(" ")
            try:
                print(json.dumps(await client.request(op, json.loads(body or "{}")), ensure_ascii=False))
            except (Fault, ValueError) as exc:
                print(f"Error: {exc}")
    finally:
        if event_task:
            event_task.cancel()
        await client.close()


def main():
    parser = argparse.ArgumentParser(description="Interactive ROKI NG protocol client")
    parser.add_argument("--robot", required=True)
    parser.add_argument("--port", type=int, default=8093)
    parser.add_argument("--control", action="store_true")
    try:
        asyncio.run(cli(parser.parse_args()))
    except KeyboardInterrupt:
        pass


if __name__ == "__main__":
    main()
