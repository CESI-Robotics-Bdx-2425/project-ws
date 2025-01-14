# web/webserver.py
import json
import threading
import asyncio
import websockets
from typing import Set
from aiohttp import web
import rospy
import os

class WebServer():
    def __init__(self, host='0.0.0.0'):
        # Initialisation du noeud ROS
        rospy.init_node('webserver', anonymous=True)
        rospy.sleep(2)

        self.host = host
        self.http_port = rospy.get_param('webserver_port')
        self.ws_port = rospy.get_param('websocket_port')
        self.running = True
        self.websocket_connections: Set[websockets.WebSocketServerProtocol] = set()
        self.loop = None
    
    async def broadcast_message(self, message: dict):
        """Envoie un message à tous les clients WebSocket connectés"""
        if self.websocket_connections:
            websockets_to_remove = set()
            message_str = json.dumps(message)
            for websocket in self.websocket_connections:
                try:
                    await websocket.send(message_str)
                except websockets.exceptions.ConnectionClosed:
                    websockets_to_remove.add(websocket)
            
            # Nettoyer les connexions fermées
            self.websocket_connections -= websockets_to_remove

    def send_message(self, message: dict):
        """Méthode thread-safe pour envoyer un message depuis un autre thread"""
        if self.loop:
            asyncio.run_coroutine_threadsafe(self.broadcast_message(message), self.loop)
    
    async def handle_websocket(self, websocket: websockets.WebSocketServerProtocol, path: str):
        """Gère les connexions WebSocket"""
        try:
            print("Nouveau client WebSocket connecté")
            self.websocket_connections.add(websocket)
            await websocket.send(json.dumps({"type": "connection", "status": "connected"}))
            
            # Maintenir la connexion active
            await websocket.wait_closed()
            
        except websockets.exceptions.ConnectionClosed:
            print("Client WebSocket déconnecté")
        finally:
            self.websocket_connections.remove(websocket)

    async def handle_index(self, request):
        """Gère la route pour la page d'index"""
        return web.FileResponse('templates/index.html')

    async def run_server(self):
        """Démarre les serveurs HTTP et WebSocket"""
        # Créer l'application web
        app = web.Application()
        app.router.add_get('/', self.handle_index)
        
        # Configurer le serveur HTTP
        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, self.host, self.http_port)
        
        # Démarrer les serveurs
        await site.start()
        print(f"Serveur HTTP démarré sur http://{self.host}:{self.http_port}")
        
        ws_server = await websockets.serve(self.handle_websocket, self.host, self.ws_port)
        print(f"Serveur WebSocket démarré sur ws://{self.host}:{self.ws_port}")
        
        try:
            while self.running:
                await asyncio.sleep(1)
        finally:
            # Nettoyage
            await runner.cleanup()
            ws_server.close()
            await ws_server.wait_closed()
    
    def stop(self):
        """Arrête le serveur"""
        self.running = False
        if self.loop:
            self.loop.call_soon_threadsafe(self.loop.stop)

    def run(self):
        """Méthode principale du thread"""
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        try:
            self.loop.run_until_complete(self.run_server())
        except Exception as e:
            print(f"Erreur dans le serveur Web : {e}")
        finally:
            self.loop.run_until_complete(self.cleanup())
            self.loop.close()

    
    async def cleanup(self):
        """Nettoie les ressources du serveur"""
        for ws in self.websocket_connections:
            await ws.close()
        self.websocket_connections.clear()

if __name__ == "__main__":
    try:
        serv = WebServer()
        serv.run()
    except rospy.ROSInterruptException:
        serv.stop()
        serv.cleanup()
        exit()