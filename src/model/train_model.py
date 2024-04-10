import os
import pandas as pd
from sklearn.model_selection import train_test_split
from transformers import BertTokenizer, BertForSequenceClassification, Trainer, TrainingArguments
from sklearn.metrics import accuracy_score
import matplotlib.pyplot as plt
import torch
import json

# 设置CUDA_LAUNCH_BLOCKING
os.environ['CUDA_LAUNCH_BLOCKING'] = '1'

# 加载数据
data_path = '/home/sunny/catkin_ws/src/project/src/data.json'
if not os.path.exists(data_path):
    raise FileNotFoundError(f"cannot find：{data_path}")

data = pd.read_json(data_path)
df = pd.DataFrame(data)

# 检查和修复标签
unique_labels = df['label'].unique()
label_to_idx = {label: idx for idx, label in enumerate(unique_labels)}
df['label'] = df['label'].map(label_to_idx)

# 保存 label_to_idx 字典
with open('/home/sunny/catkin_ws/src/project/src/results/label_to_idx.json', 'w') as f:
    json.dump(label_to_idx, f)

# 分割数据
train_df, temp_df = train_test_split(df, test_size=0.3)
val_df, test_df = train_test_split(temp_df, test_size=0.5)

# 初始化模型和分词器
model_name = 'bert-base-uncased'
tokenizer = BertTokenizer.from_pretrained(model_name)
model = BertForSequenceClassification.from_pretrained(model_name, num_labels=len(unique_labels))

# 确定设备并将模型移动到该设备
device = torch.device('cuda') if torch.cuda.is_available() else torch.device('cpu')
model.to(device)

class SimpleDataset:
    def __init__(self, encodings, labels):
        self.encodings = encodings
        self.labels = labels

    def __getitem__(self, idx):
        item = {key: val[idx].clone().detach().to(device) for key, val in self.encodings.items()}
        item['labels'] = torch.tensor(self.labels[idx], dtype=torch.long).to(device)
        return item

    def __len__(self):
        return len(self.labels)

# 构建数据集对象
train_encodings = tokenizer(train_df['text'].tolist(), truncation=True, padding=True, return_tensors='pt')
val_encodings = tokenizer(val_df['text'].tolist(), truncation=True, padding=True, return_tensors='pt')

train_dataset = SimpleDataset(train_encodings, train_df['label'].tolist())
val_dataset = SimpleDataset(val_encodings, val_df['label'].tolist())

# 训练模型
training_args = TrainingArguments(
    output_dir='./results',
    num_train_epochs=100,
    per_device_train_batch_size=8,
    per_device_eval_batch_size=8,
    logging_dir='./logs',
    logging_steps=200,
    do_train=True,
    do_eval=True,
    evaluation_strategy="steps",
    remove_unused_columns=False,  # label
)

def compute_metrics(pred):
    labels = pred.label_ids
    preds = pred.predictions.argmax(-1)
    acc = accuracy_score(labels, preds)
    return {
        'accuracy': acc,
    }

trainer = Trainer(
    model=model,
    args=training_args,
    train_dataset=train_dataset,
    eval_dataset=val_dataset,
    tokenizer=tokenizer,
    compute_metrics=compute_metrics,
    optimizers=(torch.optim.AdamW(model.parameters(), lr=5e-5), None)  # AdamW optimizer
)

trainer.train()

# Save the trained model
model.save_pretrained('./results')

# extract the training loss
train_loss = [log['loss'] for log in trainer.state.log_history if 'loss' in log]

# extract the validation loss
eval_loss = [log['eval_loss'] for log in trainer.state.log_history if 'eval_loss' in log]

# plot
plt.figure(figsize=(10, 7))
plt.plot(train_loss, label='Training Loss')
if eval_loss:
    plt.plot(eval_loss, label='Validation Loss')
plt.xlabel('Step')
plt.ylabel('Loss')
plt.title('Training and Validation Loss')
plt.legend()
plt.show()
