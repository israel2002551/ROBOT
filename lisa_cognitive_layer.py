#!/usr/bin/env python3
"""
LISA (Learning Intelligent Servo-balanced Assistant)
Layer 1: Cognitive Processing Layer (Raspberry Pi 2)

Features:
- Local wake-word / speech recognition using Vosk.
- Speculative LLM execution using Groq API (Llama-3.3-70b-specdec) with chat history.
- Multimodal Vision support using Groq API (Llama-3.2-11b-vision-preview).
- Real-time face tracking using OpenCV Haar Cascade, sending yaw offsets dynamically.
- Dynamic movement parsing (walk, idle, dance, wave, bow, hero).
- Google Text-to-Speech client, transcoding output to raw 16kHz Mono 16-bit PCM.
- High-speed UART streaming (500000 baud) of PCM audio (64-byte chunks) and JSON movement commands.
- Non-blocking OpenCV camera capture with automatic fallback if camera is disconnected.
"""

import os
import sys
import time
import json
import queue
import threading
import subprocess
import base64
import serial
import requests

# Required packages: pip install vosk pyaudio gTTS requests pyserial opencv-python

try:
    import pyaudio
except ImportError:
    print("Warning: pyaudio not found. Audio input will not work without it.")
try:
    from vosk import Model, KaldiRecognizer
except ImportError:
    print("Warning: vosk not found. Voice recognition will not work without it.")
try:
    from gtts import gTTS
except ImportError:
    print("Warning: gtts not found. Text-to-speech will not work without it.")
try:
    import cv2
except ImportError:
    print("Warning: cv2 (OpenCV) not found. Vision features will be disabled.")
    cv2 = None

# --- Configurations ---
UART_PORT = "/dev/serial0"  # UART0 on Raspberry Pi (TX Pin 8, RX Pin 10)
UART_BAUD = 500000
VOSK_MODEL_PATH = "model"   # Path to local Vosk model folder
GROQ_API_KEY = "YOUR_GROQ_API_KEY"
WAKE_WORD = "lisa"

# Global Queue for Serial Transmit
serial_queue = queue.Queue()
camera_connected = False
frame_lock = threading.Lock()
latest_frame = None

# Conversation Memory (Sliding window of last 8 messages)
chat_history = []

# Initialize Serial Connection
try:
    ser = serial.Serial(UART_PORT, UART_BAUD, timeout=1)
    print(f"UART connection established on {UART_PORT} at {UART_BAUD} baud.")
except Exception as e:
    print(f"Error opening serial port {UART_PORT}: {e}")
    ser = None

# --- Non-Blocking Camera Thread & Face Tracking ---
def camera_thread_run():
    global camera_connected, latest_frame, cv2
    if cv2 is None:
        return
        
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("System Health Warning: Camera hardware disconnected or not found. Falling back to audio-only mode.")
        camera_connected = False
        return

    camera_connected = True
    print("Camera initialized successfully. Face tracking active.")

    # Load Haar Cascade for Face Detection
    face_cascade = cv2.CascadeClassifier(cv2.data.haarcascades + 'haarcascade_frontalface_default.xml')
    last_track_time = 0
    last_face_seen_time = time.time()
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Camera read error. Gracefully switching to audio-only mode.")
            camera_connected = False
            break
        
        with frame_lock:
            latest_frame = frame

        # Real-time face tracking computation
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        faces = face_cascade.detectMultiScale(gray, 1.1, 4)
        current_time = time.time()
        
        if len(faces) > 0:
            # Sort by face area size descending (focus on the closest person)
            faces = sorted(faces, key=lambda f: f[2] * f[3], reverse=True)
            x_f, y_f, w_f, h_f = faces[0]
            frame_width = frame.shape[1]
            face_center_x = x_f + w_f / 2
            offset_x = (face_center_x - frame_width / 2) / (frame_width / 2)
            track_percentage = int(offset_x * 100) # scale to -100 to 100
            
            # Rate limit serial tracking output to 10Hz
            if current_time - last_track_time > 0.1:
                serial_queue.put(f'{{"track_x": {track_percentage}}}')
                last_track_time = current_time
            last_face_seen_time = current_time
        else:
            # If no face is seen for 1.5 seconds, slowly tell Master to center the head/body
            if current_time - last_face_seen_time > 1.5:
                serial_queue.put('{"track_x": 0}')
                last_face_seen_time = current_time
        
        time.sleep(0.03)  # ~30 FPS
        
    cap.release()

# --- Serial Sender Thread ---
def serial_sender_run():
    if ser is None:
        return
    while True:
        item = serial_queue.get()
        if isinstance(item, bytes):
            # Send raw PCM audio chunks
            ser.write(item)
            ser.flush()
            # Control flow pacing for 16kHz 16-bit Mono (32,000 bytes/sec)
            time.sleep(0.002)
        elif isinstance(item, str):
            # Send movement or system JSON commands
            ser.write(item.encode('utf-8'))
            ser.flush()
        serial_queue.task_done()

# --- Text To Speech & Transcode Audio ---
def stream_tts_to_uart(text):
    print(f"Generating TTS for: {text}")
    try:
        # Generate standard TTS file
        tts = gTTS(text=text, lang='en', slow=False)
        tts.save("temp.mp3")

        # Convert to Raw PCM 16kHz, 16-bit, Mono using ffmpeg
        process = subprocess.Popen(
            ['ffmpeg', '-y', '-i', 'temp.mp3', '-f', 's16le', '-acodec', 'pcm_s16le', '-ar', '16000', '-ac', '1', '-'],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL
        )

        # Stream PCM bytes in 64-byte chunks
        while True:
            chunk = process.stdout.read(64)
            if not chunk:
                break
            serial_queue.put(chunk)
            
        process.wait()
        print("TTS Streaming Completed.")
    except Exception as e:
        print(f"Error in TTS transcoding/streaming: {e}")

# --- Groq Speculative & Multimodal LLM Pipeline ---
def query_groq(prompt):
    global chat_history
    url = "https://api.groq.com/openai/v1/chat/completions"
    headers = {
        "Authorization": f"Bearer {GROQ_API_KEY}",
        "Content-Type": "application/json"
    }
    
    # Check if this is a vision-based request
    is_vision_query = any(word in prompt.lower() for word in ["look", "see", "show", "what is", "describe", "camera", "identify"])
    
    base64_image = None
    if is_vision_query and camera_connected:
        print("Vision query detected! Capturing latest frame...")
        with frame_lock:
            if latest_frame is not None:
                ret, buffer = cv2.imencode('.jpg', latest_frame)
                if ret:
                    base64_image = base64.b64encode(buffer).decode('utf-8')
    
    # Common system prompt defining LISA's personality and movement commands
    system_prompt = (
        "You are LISA, a bipedal robot companion. You can see objects via your camera, have conversations, "
        "and perform actions. Respond briefly, warmly, and interactively. Maintain context from previous turns. "
        "You can execute movements when asked. If the user asks you to perform an action, append a JSON command tag "
        "at the end of your response text using this exact syntax: ||{\"move\": \"<command>\", \"speed\": <value>}||. "
        "Available commands:\n"
        "- 'walk': walk forward (speed: 50-100)\n"
        "- 'dance': sway legs side-to-side and compress knees (speed: 70-100)\n"
        "- 'wave': lean body and wave the left arm servo (speed: 40-70)\n"
        "- 'bow': execute a respectful bow pose (speed: 50)\n"
        "- 'hero': execute a dramatic superhero landing pose (speed: 80)\n"
        "- 'idle': stand still in neutral calibration (speed: 0)\n"
        "Example: 'I would love to dance for you! ||{\"move\": \"dance\", \"speed\": 90}||'"
    )

    messages = [{"role": "system", "content": system_prompt}]
    
    # Append sliding window chat history
    for msg in chat_history:
        messages.append({"role": msg["role"], "content": msg["content"]})
        
    if base64_image:
        print("Querying Groq Multimodal Vision (Llama-3.2-11b)...")
        # Final message contains the prompt and image payload
        messages.append({
            "role": "user",
            "content": [
                {"type": "text", "text": prompt},
                {
                    "type": "image_url",
                    "image_url": {
                        "url": f"data:image/jpeg;base64,{base64_image}"
                    }
                }
            ]
        })
        payload = {
            "model": "llama-3.2-11b-vision-preview",
            "messages": messages,
            "temperature": 0.7
        }
    else:
        # Fallback to Llama-3.3-70b-versatile
        print("Querying Groq Engine (Llama-3.3-70b-versatile)...")
        if is_vision_query:
            prompt = "[Note: Camera is disconnected or unavailable. Respond text-only and mention your camera is offline.] " + prompt
            
        messages.append({"role": "user", "content": prompt})
        payload = {
            "model": "llama-3.3-70b-versatile",
            "messages": messages,
            "temperature": 0.7
        }

    try:
        response = requests.post(url, json=payload, headers=headers, timeout=5)
        if response.status_code == 200:
            result = response.json()
            ai_text = result['choices'][0]['message']['content']
            
            # Store turn in chat history (stripped of JSON tag to avoid polluting next text context)
            clean_text = ai_text.split("||")[0].strip()
            chat_history.append({"role": "user", "content": prompt})
            chat_history.append({"role": "assistant", "content": clean_text})
            
            # Clamp history to last 8 entries (4 turns)
            if len(chat_history) > 8:
                chat_history = chat_history[-8:]
                
            return ai_text
    except Exception as e:
        print(f"Error querying Groq: {e}")
    return "I had trouble connecting to the cloud server."

# --- Main Cognitive Loop ---
def main():
    global camera_connected
    
    # Start threads
    cam_thread = threading.Thread(target=camera_thread_run, daemon=True)
    cam_thread.start()
    
    sender_thread = threading.Thread(target=serial_sender_run, daemon=True)
    sender_thread.start()

    # Verify Vosk Model exists
    if not os.path.exists(VOSK_MODEL_PATH):
        print(f"Vosk model folder not found at '{VOSK_MODEL_PATH}'.")
        print("Please download a small model (e.g. 'vosk-model-small-en-us-0.15') and rename/extract it here.")
        sys.exit(1)

    model = Model(VOSK_MODEL_PATH)
    rec = KaldiRecognizer(model, 16000)

    p = pyaudio.PyAudio()
    stream = p.open(format=pyaudio.paInt16, channels=1, rate=16000, input=True, frames_per_buffer=8000)
    stream.start_stream()

    print("Ambient auditory loop active. Listening...")
    
    while True:
        if ser is not None and ser.is_open:
            data = ser.read(2048) # Read smaller packets at 500000 baud
        else:
            data = stream.read(4000, exception_on_overflow=False)
            
        if len(data) == 0:
            continue
            
        if rec.AcceptWaveform(data):
            result = json.loads(rec.Result())
            text = result.get("text", "")
            
            if text:
                print(f"Parsed locally: {text}")
                
                # Check for Wake Word
                if WAKE_WORD in text.lower():
                    prompt = text.lower().split(WAKE_WORD)[-1].strip()
                    if not prompt:
                        prompt = "Hello LISA"
                    
                    response_text = query_groq(prompt)
                    print(f"LISA Response: {response_text}")
                    
                    if "||" in response_text:
                        parts = response_text.split("||")
                        voice_part = parts[0]
                        command_json = parts[1].strip()
                        
                        # Forward JSON command immediately to UART
                        serial_queue.put(command_json)
                    else:
                        voice_part = response_text
                    
                    stream_tts_to_uart(voice_part)

if __name__ == "__main__":
    main()
