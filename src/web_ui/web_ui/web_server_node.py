import rclpy
from rclpy.node import Node
from flask import Flask, send_from_directory, Response
from ament_index_python.packages import get_package_share_directory
import os
import threading

# Erstelle die Flask-App
app = Flask(__name__)

# Finde den Pfad zu unseren installierten UI-Dateien
package_path = get_package_share_directory('web_ui')
ui_files_path = os.path.join(package_path, 'ui_files')

# Route für die Startseite ("/")
@app.route('/')
def index():
    # Sende die index.html
    return send_from_directory(ui_files_path, 'index.html')

# Route für alle anderen Dateien (CSS, JS, Bilder etc.)
@app.route('/<path:filename>')
def serve_files(filename):
    # Sende die angeforderte Datei aus dem ui_files Ordner
    return send_from_directory(ui_files_path, filename)

# Eine einfache /health Route, um zu prüfen, ob der Server läuft
@app.route('/health')
def health_check():
    return Response("OK", status=200)

class WebServerNode(Node):
    """
    Dieser ROS-Node startet einen Flask-Webserver in einem separaten Thread,
    um die UI-Dateien bereitzustellen.
    """
    def __init__(self):
        super().__init__('web_server_node')
        self.get_logger().info(f"Suche UI-Dateien in: {ui_files_path}")
        
        # Starte Flask in einem eigenen Thread, damit es rclpy.spin() nicht blockiert
        self.server_thread = threading.Thread(target=self.run_server)
        self.server_thread.daemon = True
        self.server_thread.start()

    def run_server(self):
        self.get_logger().info(f'Starte Webserver')
        try:
            #host='0.0.0.0' macht den Server im gesamten Netzwerk erreichbar
            app.run(host='0.0.0.0', port=8000, debug=False)
            #app.run(host='192.168.0.153', port=8000, debug=False)
            #app.run(host='rovo.local', port=8000, debug=False)
        except OSError as e:
            self.get_logger().error(f"Flask-Server konnte nicht gestartet werden: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    web_server_node = WebServerNode()
    
    # rclpy.spin() hält den Node am Leben
    try:
        rclpy.spin(web_server_node)
    except KeyboardInterrupt:
        pass
    finally:
        web_server_node.get_logger().info('Webserver wird heruntergefahren.')
        web_server_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()