import asyncio
import websockets

# 현재 연결된 클라이언트 목록
connected_clients = set()

async def signaling_server(websocket, path):
    """WebRTC 시그널링 서버 역할 수행"""
    connected_clients.add(websocket)
    try:
        async for message in websocket:
            print(f"Received: {message}")
            # 모든 클라이언트에게 메시지 전달 (Broadcast)
            for client in connected_clients:
                if client != websocket:
                    await client.send(message)
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        connected_clients.remove(websocket)

# WebSocket 서버 실행 (localhost:9000)
start_server = websockets.serve(signaling_server, "0.0.0.0", 9000)

print("WebRTC Signaling Server started on ws://0.0.0.0:9000")
asyncio.get_event_loop().run_until_complete(start_server)
asyncio.get_event_loop().run_forever()

