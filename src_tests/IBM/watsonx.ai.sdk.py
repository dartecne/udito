#
# pip install ibm-watsonx-ai
#

from ibm_watsonx_ai import APIClient
from ibm_watsonx_ai import Credentials
from ibm_watsonx_ai.foundation_models import ModelInference

import json

credentials = Credentials(
                   url = "https://eu-de.ml.cloud.ibm.com",
                   api_key = "TP8qCL22zUpXkw2mHMtoAwJJZqM9ZBzKSwV_kPR60E0S"
                  )

client = APIClient(credentials)


# === Configuración ===
api_key = "TP8qCL22zUpXkw2mHMtoAwJJZqM9ZBzKSwV_kPR60E0S"  # la de tu servicio watsonx.ai
project_id = "b65c1eaa-91ac-4520-9a79-ce25df3876a0"  # el ID del proyecto watsonx que creaste
region = "eu-de"

# === Credenciales y conexión ===
creds = Credentials(
    url=f"https://{region}.ml.cloud.ibm.com",
    api_key=api_key
)

# === Crear el modelo (Generative Model) ===
model = ModelInference(
    model_id="meta-llama/llama-3-3-70b-instruct",
    credentials=creds,
    project_id=project_id
)

# === Prompt y generación ===
#prompt = "Explícame la diferencia entre inteligencia artificial débil y fuerte."
prompt = "Cómo funciona la fotosíntesis"

response = model.generate_text(
    prompt=prompt)

# === Mostrar resultado ===
print(response)
print(json.dumps(response, indent=2))
