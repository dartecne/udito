from transformers import pipeline

synthesizer = pipeline("text-to-speech", "suno/bark")

synthesizer("Look I am generating speech in three lines of code!")
