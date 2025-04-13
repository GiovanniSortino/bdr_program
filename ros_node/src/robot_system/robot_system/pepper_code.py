import rclpy
from rclpy.node import Node
import qi
import argparse
import sys
import time
import paramiko
import os
from std_msgs.msg import Bool,String
from .WhisperHugging import WhisperHugging


class PepperSpeakerNode(Node):
    def __init__(self, ip, port):
        super().__init__('pepper_speaker_node')
        
        self.get_logger().info(f"Connessione a Pepper su {ip}:{port}")
        try:
            
            #Parametri di connessione
            self.ip = ip
            self.session = self.set_connection(ip, port)

            #Speak
            self.tts = self.session.service("ALTextToSpeech")
            self.tts.setLanguage("Italian")
            
            # Moduli per riconoscimento vocale
            self.sound_detect_service = self.session.service("ALSoundDetection")
            self.leds_service = self.session.service("ALLeds")
            self.sound_detect_service.setParameter("Sensitivity", 0.8)

            self.audio_service = self.session.service("ALAudioRecorder")
            self.leds_service = self.session.service("ALLeds")
            self.memory = self.session.service("ALMemory")

            self.last_index=self.memory.getData("ALTextToSpeech/Status")[0]
            self.is_recognizing = False

            #Publisher
            self.isSpeaking_pub=self.create_publisher(Bool,'is_speaking',10)
            self.transcription_pub=self.create_publisher(String,'transcription',10)

            #Subscription
            self.subscription = self.create_subscription(String, 'pepper_interaction', self.speak_callback, 10)
            self.subscription_record = self.create_subscription(Bool, 'record', self.record_callback, 10)

            self.create_timer(0.5, self.check_speaking)

        except Exception as e:
            self.get_logger().error(f"Errore nella connessione a Pepper: {e}")
            sys.exit(1)

    def set_connection(self, ip, port):
        session = qi.Session()
        try:
            session.connect(f"tcp://{ip}:{port}")
        except RuntimeError:
            self.get_logger().error(f"Can't connect to Naoqi at ip \"{ip}\" on port {port}.\n"
                                    "Please check your script arguments.")
            sys.exit(1)
        return session

    def speak_callback(self, msg):
        self.get_logger().info(f"Sono dentro Speak CallBack")
        text = msg.data
        self.tts.say(text)
        

    def record_callback(self, msg):
        self.get_logger().warn("SONO DENTRO LA CALLBACK RECORD")

        if getattr(self, "is_recording", False):
            self.get_logger().warn("Registrazione già in corso, callback ignorato.")
            return

        self.is_recording = True

        try:
            self.audio_service.stopMicrophonesRecording()
            self.get_logger().info("Registrazione precedente interrotta.")
        except Exception:
            self.get_logger().warn("Nessuna registrazione attiva da fermare.")
            
        channels = [1, 1, 1, 1]
        audio_format = "wav"
        sample_rate = 16000
        output_file_robot = "/home/nao/audio_record_unipa/recording.wav"
        
        self.sound_detect_service.subscribe("Audio Detection")
        self.audio_service.startMicrophonesRecording(output_file_robot, audio_format, sample_rate, channels)
        self.get_logger().info("(Pepper Interaction: Avvio microfoni")
        self.set_led(True)
        time.sleep(0.3)
        while not self.is_recognizing:
            time.sleep(0.3)
            
            if  self.memory.getData("SoundDetected")[0][1]==1:
                self.get_logger().info("Avvio registrazione...")
                
                self.is_recognizing=True
                
        # Attendere la fine della registrazione
        while self.is_recognizing:
            time.sleep(2)
            if  self.memory.getData("SoundDetected")[0][1]==0:
                self.is_recognizing=False
                    # Terminare la registrazione
                self.audio_service.stopMicrophonesRecording()
                self.set_led(False)
                self.sound_detect_service.unsubscribe("Audio Detection")
                self.get_logger().info(f"Registrazione terminata e salvata in: {output_file_robot}")
    

        path_ros_ws=os.path.join(os.path.abspath(__file__).split("/install")[0])
     
        # Trasferire il file al PC
        local_output_file = os.path.join(path_ros_ws,"src/audio/recording.wav")
        
        ssh = paramiko.SSHClient()
        ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
        ssh.connect(self.ip, username='nao', password='nao')#'192.168.0.161'

        sftp = ssh.open_sftp()
        sftp.get(output_file_robot, local_output_file)
        sftp.close()
        ssh.close()
        self.get_logger().info("File trasferito con successo!")
        
        '''res=String()

        res.data=local_output_file'''
        self.get_logger().info("RECORD_CALLBACK")
        self.whisper = WhisperHugging()

        transcription=self.whisper.transcribe_audio("src/audio/recording.wav") #verificare se invece del percroso posso mettere local_outpu_file
        self.get_logger().info(f"TRASCRIZIONE EFFETTUATA {transcription}")

        res=String()
        res.data=transcription
        
        self.transcription_pub.publish(res) 

    def check_speaking(self):
        msg=Bool()
        status=self.memory.getData("ALTextToSpeech/Status")
        if status[1]=="done" and status[0]!=self.last_index:
            msg.data=False
            self.isSpeaking_pub.publish(msg)
        else :
            msg.data=True
            self.isSpeaking_pub.publish(msg)
        self.get_logger().info(f"CHECK_SPEAKING: {msg.data}")
        self.last_index=status[0]

    def set_led(self, on):
            names = [
            "Face/Led/Green/Left/0Deg/Actuator/Value",
            "Face/Led/Green/Left/45Deg/Actuator/Value",
            "Face/Led/Green/Left/90Deg/Actuator/Value",
            "Face/Led/Green/Left/135Deg/Actuator/Value",
            "Face/Led/Green/Left/180Deg/Actuator/Value",
            "Face/Led/Green/Left/225Deg/Actuator/Value",
            "Face/Led/Green/Left/270Deg/Actuator/Value",
            "Face/Led/Green/Left/315Deg/Actuator/Value",

            "Face/Led/Green/Right/0Deg/Actuator/Value",
            "Face/Led/Green/Right/45Deg/Actuator/Value",
            "Face/Led/Green/Right/90Deg/Actuator/Value",
            "Face/Led/Green/Right/135Deg/Actuator/Value",
            "Face/Led/Green/Right/180Deg/Actuator/Value",
            "Face/Led/Green/Right/225Deg/Actuator/Value",
            "Face/Led/Green/Right/270Deg/Actuator/Value",
            "Face/Led/Green/Right/315Deg/Actuator/Value"]

            self.leds_service.createGroup("eyes",names)
            # Switch the new group on
            if on==True:
                self.leds_service.off("FaceLeds")
                self.leds_service.on("eyes")
            elif on==False:
                self.leds_service.off("eyes")
                self.leds_service.on("FaceLeds")


def main(args=None):
    parser = argparse.ArgumentParser()
    parser.add_argument("--ip", type=str, default="192.168.0.101",
                        help="Indirizzo IP del robot.")
    parser.add_argument("--port", type=int, default=9559,
                        help="Porta Naoqi.")
    parsed_args, remaining = parser.parse_known_args()

    rclpy.init(args=remaining)
    node = PepperSpeakerNode(parsed_args.ip, parsed_args.port)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
