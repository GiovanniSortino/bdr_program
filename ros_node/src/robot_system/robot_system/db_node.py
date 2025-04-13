import rclpy
from rclpy.node import Node
from neo4j import GraphDatabase
from std_msgs.msg import String 
import json


class Neo4jNode(Node):
    def __init__(self):
        super().__init__('neo4j_node')

        self.uri = "bolt://localhost:7687"  
        self.user = "neo4j"  
        self.password = "12345678" 

        self.get_logger().info(f"Connecting to Neo4j at {self.uri}...")

        try:
            self.driver = GraphDatabase.driver(self.uri, auth=(self.user, self.password))
            self.get_logger().info("Connected to Neo4j successfully.")
        except Exception as e:
            self.get_logger().error(f"Failed to connect to Neo4j: {str(e)}")
            return
        
        #Publisher
        self.response_publisher = self.create_publisher(String, 'db_response', 10)

        #Subscription
        self.subscription = self.create_subscription(String, 'db_request', self.listener_callback,10)
        self.subscription  # Necessario per evitare che venga eliminato dal garbage collector


    def listener_callback(self, msg):
        command = msg.data.strip().lower()
        self.get_logger().info(f"Received command: {command}")

        command_to_query = {
            "mostra_anziani": "MATCH (a:Anziano) RETURN a",
            "mostra_robot": "MATCH (r:AssistenteRobot) RETURN r",
            "conta_anziani": "MATCH (a:Anziano) RETURN count(a)",
            "cancella_tutto": "MATCH (n) DETACH DELETE n",
            "get_db": "MATCH (n) RETURN n",
            "get_beliefs": "MATCH (a:Anziano) RETURN a.nome"
        }

        if command in command_to_query:
            query = command_to_query[command]
            self.get_logger().info(f"Eseguo la query per '{command}'")

            result = self.run_query(query)
            json_result = json.dumps(result)

            msg_out = String()
            msg_out.data = json_result
            self.response_publisher.publish(msg_out)

            self.get_logger().info(f"Risposta pubblicata: {json_result}")
        else:
            self.get_logger().warn(f"Comando non riconosciuto: {command}")

    def run_query(self, query):
        self.get_logger().info(f"Running query: {query}")
        try:
            with self.driver.session() as session:
                result = session.run(query)

                records = [record.data() for record in result]
                return records

        except Exception as e:
            self.get_logger().error(f"Failed to run query: {str(e)}")
            return f"Errore: {str(e)}"

    def __del__(self):
        if hasattr(self, 'driver') and self.driver:
            self.get_logger().info("Closing the Neo4j connection.")
            self.driver.close()

def main(args=None):
    rclpy.init(args=args)

    neo4j_node = Neo4jNode()

    rclpy.spin(neo4j_node)

    neo4j_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
