import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import asyncio
import threading
import os

from langchain_core.prompts import ChatPromptTemplate
from langchain_groq import ChatGroq

class LLM_node(Node):
    def __init__(self):
        super().__init__('groq_assistant_node')
        self.get_logger().info("Nodo LLM_model avviato, in ascolto su LLM_request")

        #Publisher
        self.LLM_response_pub = self.create_publisher(String, 'LLM_response', 10)

        #Subscription
        self.LLM_request_sub = self.create_subscription(String, 'LLM_request', self.listener_callback, 10)

        self.chat = ChatGroq(
            temperature=0,
            groq_api_key=os.getenv("LLM_API_TOKEN"),
            model_name="deepseek-r1-distill-llama-70b"
        )
        self.prompt = ChatPromptTemplate.from_messages([
            ("system", "{system}"),
            ("human", "{text}")
        ])
        self.chain = self.prompt | self.chat

        self.loop = asyncio.new_event_loop()
        threading.Thread(target=self.start_loop, daemon=True).start()

    def start_loop(self):
        asyncio.set_event_loop(self.loop)
        self.loop.run_forever()

    def listener_callback(self, msg):
        data = json.loads(msg.data)
        self.get_logger().info(f"Messaggio ricevuto: {data}")

        # Esegui la coroutine nel loop in background
        asyncio.run_coroutine_threadsafe(self.ask_groq(data), self.loop)

    async def ask_groq(self, data):
        try:
            response = await self.chain.ainvoke({
                "system": data["system"],
                "text": data["text"]
            })
            answer = response.content
            self.get_logger().info(f"Risposta del modello: {answer}")

            # Trova la parte dopo il tag </think>
            end_tag = "</think>"
            if end_tag in answer:
                # Estrai tutto dopo il tag </think>
                answer_after_think = answer.split(end_tag, 1)[-1].strip()  # prendi la parte dopo il tag e rimuovi gli spazi
            else:
                answer_after_think = answer  # Se non c'è </think>, prendi tutto il testo

            self.get_logger().info(f"Risposta dopo il tag </think>: {answer_after_think}")

            '''# Crea il messaggio da inviare, includendo il nodo e la risposta
            message = {
                "nodo": "pepper_code",  # Puoi modificare "pepper_code" se necessario
                "text": answer_after_think
            }

            # Serializza il messaggio in una stringa JSON
            json_message = json.dumps(message)'''

            # Crea un oggetto String per il messaggio ROS
            msg_out = String()
            msg_out.data = answer_after_think  # Assegna la stringa JSON al messaggio ROS

            # Pubblica il messaggio
            self.LLM_response_pub.publish(msg_out)

        except Exception as e:
            self.get_logger().error(f"Errore durante l'invocazione del modello: {e}")

def main(args=None):
    rclpy.init(args=args)
    
    node = LLM_node()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
