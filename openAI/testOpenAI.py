import os
from openai import OpenAI
OPENAI_API_KEY="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"
client = OpenAI(
#    api_key=os.environ.get("OPENAI_API_KEY"), #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"),  # This is the default and can be omitted
api_key=OPENAI_API_KEY
)

chat_completion = client.chat.completions.create(
    messages=[
        {
            "role": "user",
            "content": "Say this is a test",
        }
    ],
    model="gpt-4o-mini",
)