from pathlib import Path
from openai import OpenAI

OPENAI_API_KEY="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"

client = OpenAI(api_key=OPENAI_API_KEY)
speech_file_path = Path(__file__).parent / "speech.mp3"
response = client.audio.speech.create(
    model="tts-1",
    voice="alloy",
    input="Today is a wonderful day to build something people love!",
)
response.stream_to_file(speech_file_path)