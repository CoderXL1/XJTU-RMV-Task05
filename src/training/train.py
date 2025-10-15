import torch
import torch.nn as nn
import torch.optim as optim
from torch.utils.data import DataLoader
from torchvision import datasets, transforms
import os
import random

# -----------------------------
# 配置参数
# -----------------------------
IMG_SIZE = (48, 32)   # 训练输入尺寸 (H,W)
BATCH_SIZE = 64
EPOCHS = 30
LR = 1e-3
DATASET_PATH = "dataset_split"
DEVICE = "cpu" # torch.device("cuda" if torch.cuda.is_available() else "cpu")
MODEL_SAVE_PATH = "best_model.pth"

# -----------------------------
# 模型定义 (Tiny LeNet)
# -----------------------------
class TinyLeNet(nn.Module):
    def __init__(self, num_classes):
        super().__init__()
        self.conv1 = nn.Conv2d(1, 16, 5, padding=2)  # preserve
        self.pool  = nn.MaxPool2d(2,2)
        self.conv2 = nn.Conv2d(16, 32, 5, padding=2)
        self.fc1   = nn.Linear(32 * (IMG_SIZE[0]//4) * (IMG_SIZE[1]//4), 128)
        self.fc2   = nn.Linear(128, num_classes)

    def forward(self, x):
        x = self.pool(torch.relu(self.conv1(x)))  # H/2,W/2
        x = self.pool(torch.relu(self.conv2(x)))  # H/4,W/4
        x = x.view(x.size(0), -1)
        x = torch.relu(self.fc1(x))
        return self.fc2(x)

def train():

    # -----------------------------
    # 数据增强和预处理
    # -----------------------------
    train_transforms = transforms.Compose([
        transforms.Grayscale(num_output_channels=1),
        transforms.RandomPerspective(distortion_scale=0.3, p=0.7),
        transforms.RandomAffine(degrees=15, translate=(0.1,0.1), scale=(0.8,1.2), shear=10),
        transforms.RandomRotation(15),
        transforms.Resize(IMG_SIZE),
        transforms.ToTensor(),
        transforms.Normalize((0.5,), (0.5,))
    ])

    val_transforms = transforms.Compose([
        transforms.Grayscale(num_output_channels=1),
        transforms.Resize(IMG_SIZE),
        transforms.ToTensor(),
        transforms.Normalize((0.5,), (0.5,))
    ])

    # -----------------------------
    # 数据集加载
    # -----------------------------
    train_dataset = datasets.ImageFolder(os.path.join(DATASET_PATH, "train"), transform=train_transforms)
    val_dataset   = datasets.ImageFolder(os.path.join(DATASET_PATH, "val"), transform=val_transforms)

    train_loader = DataLoader(train_dataset, batch_size=BATCH_SIZE, shuffle=True, num_workers=2)
    val_loader   = DataLoader(val_dataset, batch_size=BATCH_SIZE, shuffle=False, num_workers=2)

    num_classes = len(train_dataset.classes)
    print("Classes:", train_dataset.classes)

    model = TinyLeNet(num_classes).to(DEVICE)
    criterion = nn.CrossEntropyLoss()
    optimizer = optim.Adam(model.parameters(), lr=LR)
    scheduler = optim.lr_scheduler.ReduceLROnPlateau(optimizer, 'max', patience=3, factor=0.5)

    # -----------------------------
    # 训练循环
    # -----------------------------
    best_val_acc = 0.0
    optimal_cnt = 0

    for epoch in range(1, EPOCHS+1):
        model.train()
        running_loss = 0.0
        for imgs, labels in train_loader:
            imgs, labels = imgs.to(DEVICE), labels.to(DEVICE)
            optimizer.zero_grad()
            outputs = model(imgs)
            loss = criterion(outputs, labels)
            loss.backward()
            optimizer.step()
            running_loss += loss.item() * imgs.size(0)

        train_loss = running_loss / len(train_loader.dataset)

        # 验证
        model.eval()
        correct = total = 0
        with torch.no_grad():
            for imgs, labels in val_loader:
                imgs, labels = imgs.to(DEVICE), labels.to(DEVICE)
                outputs = model(imgs)
                preds = outputs.argmax(dim=1)
                correct += (preds == labels).sum().item()
                total += labels.size(0)
        val_acc = correct / total
        scheduler.step(val_acc)

        print(f"Epoch {epoch}/{EPOCHS} - Train Loss: {train_loss:.4f} - Val Acc: {val_acc:.4f}")

        # 保存最佳模型
        if val_acc > best_val_acc:
            optimal_cnt = 0
            best_val_acc = val_acc
            torch.save({
                'model_state': model.state_dict(),
                'classes': train_dataset.classes
            }, MODEL_SAVE_PATH)
            print(f"Best model saved with val acc: {best_val_acc:.4f}")
        else:
            optimal_cnt+=1
        if optimal_cnt > 5:
            print("Model is not improving, abort.")
            break

    print("Training finished. Best val acc:", best_val_acc)

if __name__ == "__main__":
    train()