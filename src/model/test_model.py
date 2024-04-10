from transformers import BertTokenizer, BertForSequenceClassification
import torch
import json

# Load the saved model and tokenizer
model_name = 'bert-base-uncased'
tokenizer = BertTokenizer.from_pretrained(model_name)
model = BertForSequenceClassification.from_pretrained('./results')

# Determine the device and move the model to that device
device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
model.to(device)

# Load the label_to_idx dictionary and create the idx_to_label dictionary
with open('./results/label_to_idx.json', 'r') as f:
    label_to_idx = json.load(f)
idx_to_label = {idx: label for label, idx in label_to_idx.items()}

# Predict the label for a new text
new_text = "Hi, robot. I want to text to my friend via my cell phone."
new_text_encoding = tokenizer(new_text, truncation=True, padding=True, return_tensors='pt').to(device)
output = model(**new_text_encoding)
pred_label = output.logits.argmax(-1).item()

# Convert the predicted index to label
#pred_label_text = idx_to_label[str(pred_label)]  # Ensure the key is a string when accessing the dictionary
pred_label_text = idx_to_label[pred_label]  # Use pred_label directly as the key


print(f"The predicted label is: {pred_label_text}")
