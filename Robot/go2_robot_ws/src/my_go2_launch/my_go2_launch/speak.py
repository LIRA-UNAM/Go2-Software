#!/usr/bin/env python3
# import rclpy
# from rclpy.node import Node
# from go2_webrtc_driver.webrtc_driver import Go2WebRTCConnection, WebRTCConnectionMethod
# from go2_webrtc_driver.webrtc_audiohub import WebRTCAudioHub

# import os
# import json
# import asyncio

# class AudioPlayer(Node):
#     def __init__(self):
#         super().__init__('audio_player')

#         # Configuración del archivo de audio
#         self.audio_file = "dog-barking.wav"
#         self.audio_file_path = os.path.join(os.path.dirname(__file__), self.audio_file)

#         # Crea la conexión WebRTC
#         self.conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip="192.168.123.18")

#         # Arranca la rutina asíncrona para manejar audio
#         asyncio.ensure_future(self.play_audio())

#     async def play_audio(self):
#         try:
#             await self.conn.connect()
#             self.get_logger().info("WebRTC conectado ✅")

#             audio_hub = WebRTCAudioHub(self.conn, self.get_logger())
#             self.get_logger().info("AudioHub inicializado")

#             # Obtener lista de audios
#             response = await audio_hub.get_audio_list()
#             if response and isinstance(response, dict):
#                 data_str = response.get('data', {}).get('data', '{}')
#                 audio_list = json.loads(data_str).get('audio_list', [])

#                 filename = os.path.splitext(self.audio_file)[0]
#                 existing_audio = next((audio for audio in audio_list if audio['CUSTOM_NAME'] == filename), None)

#                 if existing_audio:
#                     uuid = existing_audio['UNIQUE_ID']
#                     self.get_logger().info(f"Archivo ya existe en robot con UUID: {uuid}")
#                 else:
#                     self.get_logger().info("Subiendo archivo...")
#                     await audio_hub.upload_audio_file(self.audio_file_path)
#                     response = await audio_hub.get_audio_list()
#                     audio_list = json.loads(response.get('data', {}).get('data', '{}')).get('audio_list', [])
#                     existing_audio = next((audio for audio in audio_list if audio['CUSTOM_NAME'] == filename), None)
#                     uuid = existing_audio['UNIQUE_ID']

#                 # Reproducir
#                 self.get_logger().info(f"Reproduciendo UUID: {uuid}")
#                 await audio_hub.play_by_uuid(uuid)
#                 self.get_logger().info("Reproducción terminada ✅")

#         except Exception as e:
#             self.get_logger().error(f"Error: {e}")


# async def main():
#     print(">>> Iniciando conexión con el robot...")
#     conn = Go2WebRTCConnection(WebRTCConnectionMethod.LocalSTA, ip="192.168.137.120")
#     await conn.connect()
#     print("✅ Conexión WebRTC establecida")

#     audio_hub = WebRTCAudioHub(conn)
#     print("✅ Audio hub inicializado")

#     audio_file = "dog-barking.wav"
#     print(f">>> Subiendo archivo: {audio_file}")
#     await audio_hub.upload_audio_file(audio_file)
#     print("✅ Archivo subido")

#     audio_list = await audio_hub.get_audio_list()
#     print(f"Lista de audios en el robot: {audio_list}")

#     # Busca por nombre
#     filename = os.path.splitext(audio_file)[0]
#     existing_audio = next((a for a in audio_list['audio_list'] if a['CUSTOM_NAME'] == filename), None)

#     if existing_audio:
#         uuid = existing_audio['UNIQUE_ID']
#         print(f"✅ Archivo encontrado en robot con UUID: {uuid}")
#         print(">>> Reproduciendo...")
#         await audio_hub.play_by_uuid(uuid)
#         print("✅ Reproducción enviada")
#     else:
#         print("⚠️ El archivo no apareció en la lista del robot")


# if __name__ == '__main__':
#     asyncio.run(main())


import rclpy
from rclpy.node import Node
from unitree_go.msg import AudioData
from gtts import gTTS
import io
import numpy as np
import soundfile as sf
from scipy.signal import resample
import time

CHUNK_SIZE = 512       # tamaño de cada chunk
CHUNK_DELAY = 0.01     # segundos entre chunks
TARGET_RATE = 16000    # Hz

class TTSSpeaker(Node):
    def __init__(self):
        super().__init__('tts_speaker')
        self.publisher = self.create_publisher(AudioData, '/audioreceiver', 10)
        self.get_logger().info("Nodo TTS listo. Enviando audio al Go2...")

    def tts_to_wav(self, text):
        """Genera WAV en float32 mono 16 kHz desde texto"""
        tts = gTTS(text=text, lang='es')
        fp = io.BytesIO()
        tts.write_to_fp(fp)
        fp.seek(0)

        data, samplerate = sf.read(fp)
        if data.ndim > 1:
            data = np.mean(data, axis=1)

        # Resample a 16 kHz si es necesario
        if samplerate != TARGET_RATE:
            num_samples = int(len(data) * TARGET_RATE / samplerate)
            data = resample(data, num_samples)

        return data

    def float_to_uint8(self, data):
        """Convierte float32 (-1,1) a uint8 (0,255) centrado en 127"""
        max_val = np.max(np.abs(data))
        if max_val > 0:
            data = data / max_val * 0.9  # evita clipping
        data_uint8 = ((data + 1.0) * 127.5).clip(0, 255).astype(np.uint8)
        return data_uint8

    def send_audio(self, text):
        """Envía audio TTS al Go2 en chunks"""
        data = self.tts_to_wav(text)
        data_uint8 = self.float_to_uint8(data)

        # Dividir en chunks
        chunks = [data_uint8[i:i+CHUNK_SIZE] for i in range(0, len(data_uint8), CHUNK_SIZE)]
        for chunk in chunks:
            msg = AudioData()
            msg.data = chunk.tolist()
            self.publisher.publish(msg)
            time.sleep(CHUNK_DELAY)

        self.get_logger().info(f"Enviado audio TTS en {len(chunks)} chunks")

def main():
    rclpy.init()
    node = TTSSpeaker()

    texto = "Hola, soy tu robot Go2. Este es un audio de prueba desde ROS2."
    node.send_audio(texto)

    rclpy.spin_once(node)  # mantener nodo vivo un ciclo
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
