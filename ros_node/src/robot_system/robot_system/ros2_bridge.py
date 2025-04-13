import socket
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
import json

class SocketServerNode(Node):
    def __init__(self):
        super().__init__('socket_server_node')
    
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        host = '127.0.0.1'
        port = 5050
        self.server_socket.bind((host, port))
        self.server_socket.listen(5)
        self.is_speaking_bool = True
        self.latest_response = None


        #Publisher
        self.publisher_db = self.create_publisher(String, 'db_request', 10)
        self.publisher_pepper_interaction = self.create_publisher(String, 'pepper_interaction', 10) 
        self.publisher_LLM_request = self.create_publisher(String, 'LLM_request', 10)
        self.record_pub =self.create_publisher(Bool,"record",10)

        #Subscription
        self.subscription = self.create_subscription(String,'db_response',self.listener_callback,10)
        self.subscription_is_speaking = self.create_subscription(Bool,'is_speaking',self.is_speaking,10)
        self.subscription_transcription = self.create_subscription(String,'LLM_response',self.LLM_response,10)
        self.subscription_transcription = self.create_subscription(String,'transcription',self.transcription_callback,10)


    def listener_callback(self, msg):
        self.latest_response = msg.data
        self.get_logger().info(f"Risposta JSON ricevuta da db_response: {self.latest_response}")

    def is_speaking(self, msg):
        msg2=Bool()
        self.is_speaking_bool = msg.data
        msg2.data=True
        if not msg.data:
            self.record_pub.publish(msg2)

    def transcription_callback(self, msg):
        #Modificare quando voglio farlo passare agli agenti BDI
        self.invia_a_llm(msg)

    def LLM_response(self, msg):
        nome = msg['text']
        msg = String()
        msg.data = nome
        self.get_logger().info(f"LLM_RESPONSE")
        self.publisher_pepper_interaction.publish(msg)

    def run_server(self):
        while rclpy.ok():
            try:
                client_socket, client_address = self.server_socket.accept()
                message = client_socket.recv(1024).decode()

                self.spedisci_a_nodo(message, client_socket)

            except KeyboardInterrupt:
                self.get_logger().info("\nServer fermato manualmente.")
                break
            except Exception as e:
                self.get_logger().error(f"Errore durante la connessione: {e}")
                continue

        self.server_socket.close()

    def spedisci_a_nodo(self, message, client_socket):
        data = json.loads(message)
        nodo = data["nodo"]

        self.get_logger().info(f"Messaggio ricevuto dal client: {data}")
        self.get_logger().info(f"Nodo: {nodo}")

        if nodo == "database":
            comando = data["comando"]
            self.invia_a_db(comando)

            timeout = 20.0 
            waited = 0.0
            while self.latest_response is None and waited < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
                waited += 0.1

            if self.latest_response is not None:
                client_socket.send(self.latest_response.encode())
                self.get_logger().info("Risposta inviata al client.")
            else:
                client_socket.send(b"Timeout: nessuna risposta ricevuta dal nodo DB.")
                self.get_logger().warn("Timeout in attesa della risposta.")

            self.latest_response = None  # Reset per la prossima richiesta
            client_socket.close()
            
        elif nodo == "pepper_code":
            self.get_logger().info(f"SONO IN PEPPER CODE: {data}")
            self.invia_a_pepper_interaction(data)
            
        elif nodo == "llm_node":
            data_llm_node = {
                "system": data.get("system"),
                "text": data.get("text")
            }
            self.invia_a_llm(data_llm_node)

    def invia_a_db(self, comando):
        msg = String()
        msg.data = comando
        self.publisher_db.publish(msg)

    def invia_a_pepper_interaction(self,data):
        #Generalizzare
        nome = data['nome']
        msg = String()
        msg.data = "Ciao "+ nome + ", oggi ti assisterò io"
        self.get_logger().info(f"Dentro Pepper Code")
        self.publisher_pepper_interaction.publish(msg)

    def invia_a_llm(self, data_llm_node):
        msg = String()
        msg.data = json.dumps(data_llm_node)
        self.get_logger().info(f"Sono dentro invia_a_LLM {msg}")
        self.publisher_LLM_request.publish(msg)
        
def main(args=None):
    rclpy.init(args=args)
    server_node = SocketServerNode()

    try:
        server_node.run_server()
    except KeyboardInterrupt:
        pass

    server_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

