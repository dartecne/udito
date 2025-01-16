import soundfile
from espnet2.bin.tts_inference import Text2Speech
text2speech = Text2Speech.from_pretrained("kan-bayashi/ljspeech_joint_finetune_conformer_fastspeech2_hifigan")
speech = text2speech("foobar")["wav"]
soundfile.write("out.wav", speech.numpy(), text2speech.fs, "PCM_16")