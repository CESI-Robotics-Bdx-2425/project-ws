#!/usr/bin/env python3
import rospy, rospkg, time
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
        self.http_port = rospy.get_param('webserver_port', 8000)
        # self.ws_port = rospy.get_param('websocket_port', 8001)
        self.running = True
        
    async def handle_index(self, request):
        try:
            print("Acces au index.html")
            return web.FileResponse(f"{self.path}/src/templates/index.html")
        except Exception as e:
            rospy.logerr(f"Erreur lors de l'accès au fichier : {e}")
            return web.Response(status=500, text="Erreur interne au serveur")
    
    async def handle_roslib(self, request):
        try:
            print("Acces au roslib.min.js")
            return web.FileResponse(f"{self.path}/src/templates/roslib.min.js")
        except Exception as e:
            rospy.logerr(f"Erreur lors de l'accès au fichier : {e}")
            return web.Response(status=500, text="Erreur interne au serveur")
            
    async def run_server(self):
        """Démarre les serveurs HTTP et WebSocket"""
        # Créer l'application web
        app = web.Application()
        app.router.add_get('/', self.handle_index)
        app.router.add_get('/roslib', self.handle_roslib)
        # Configurer le serveur HTTP
        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, self.host, self.http_port)

        try:
            await site.start()
            rospy.loginfo(f"Serveur HTTP démarré sur http://{self.host}:{self.http_port}")
            while self.running:
                await asyncio.sleep(1)
        except Exception as e:
            rospy.logerr(f"Erreur dans le serveur HTTP : {e}")
        finally:
            await runner.cleanup()
            
    def run(self):
        while True:
            try:
                rospy.loginfo("Démarrage du serveur web...")
                self.loop = asyncio.new_event_loop()
                asyncio.set_event_loop(self.loop)
                self.loop.run_until_complete(self.run_server())
            except Exception as e:
                rospy.logerr(f"Le serveur a rencontré une erreur : {e}. Redémarrage dans 5 secondes.")
                time.sleep(5)
            finally:
                if self.loop:
                    self.loop.close()    

if __name__ == '__main__':
    try:
        serv = WebServer()
        serv.run()
    except rospy.ROSInterruptException:
        exit()