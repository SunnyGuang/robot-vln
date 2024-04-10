import speech_recognition as sr
from transformers import BertTokenizer, BertForSequenceClassification
import torch
import json
import pandas as pd
import os

# Load the saved model and tokenizer
model_name = 'bert-base-uncased'
tokenizer = BertTokenizer.from_pretrained(model_name)
model = BertForSequenceClassification.from_pretrained('./results')

# Determine the device and move the model to that device
device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
model.to(device)

# Load the label_to_idx dictionary and create the idx_to_label dictionary
with open('/home/sunny/catkin_ws/src/project/src/results/label_to_idx.json', 'r') as f:
    label_to_idx = json.load(f)
idx_to_label = {str(idx): label for label, idx in label_to_idx.items()}

# Initialize the recognizer
recognizer = sr.Recognizer()

# Record the audio
with sr.Microphone() as source:
    print("Please say something...")
    audio_data = recognizer.record(source, duration=10)  # Adjust duration as needed
    print("Recording completed.")

# Convert the audio to text using Google Speech-to-Text
try:
    new_text = recognizer.recognize_google(audio_data)
    print("You said: " + new_text)
except sr.UnknownValueError:
    print("Google Speech-to-Text could not understand the audio")
except sr.RequestError:
    print("Could not request results from Google Speech-to-Text; check your network connection")

# Predict the label for the new text
if new_text:
    new_text_encoding = tokenizer(new_text, truncation=True, padding=True, return_tensors='pt').to(device)
    output = model(**new_text_encoding)
    pred_label = output.logits.argmax(-1).item()

    # Convert the predicted index to label
    pred_label_text = idx_to_label[str(pred_label)]
    # print(f"The predicted label is: {pred_label_text}")

