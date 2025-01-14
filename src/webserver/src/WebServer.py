#!/usr/bin/env python3
import rospy, rospkg
import asyncio
from aiohttp import web

class WebServer():
    def __init__(self):
        # Initialisation du noeud ROS
        rospy.init_node('webserver', anonymous=True)
        rospy.sleep(2)
        
        # Init du paquet
        self.r = rospkg.RosPack()
        self.path = self.r.get_path('webserver')
        
        self.host = "0.0.0.0"
        self.http_port = rospy.get_param('webserver_port')
        self.ws_port = rospy.get_param('websocket_port')
        self.running = True
        
    async def handle_index(self, request):
        try:
            return web.FileResponse(f"{self.path}/src/templates/index.html")
        except Exception as e:
            rospy.logerr(e)
            
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
        rospy.loginfo(f"Serveur HTTP démarré sur http://{self.host}:{self.http_port}")
        
        try:
            while self.running:
                await asyncio.sleep(1)
        finally:
            # Nettoyage
            await runner.cleanup()
            
    def run(self):
        """Méthode principale du thread"""
        self.loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self.loop)
        try:
            self.loop.run_until_complete(self.run_server())
        except Exception as e:
            print(f"Erreur dans le serveur Web : {e}")
        finally:
            self.loop.close()    

if __name__ == '__main__':
    try:
        serv = WebServer()
        serv.run()
    except rospy.ROSInterruptException:
        exit()