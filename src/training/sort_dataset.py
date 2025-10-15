import os, shutil, random
from pathlib import Path

random.seed(42)
root = Path("dataset")
target_root = Path("dataset_split")
for cls in root.iterdir():
    if not cls.is_dir(): continue
    imgs = list(cls.glob("*"))
    random.shuffle(imgs)
    n = len(imgs)
    n_train, n_val = int(0.8*n), int(0.9*n)
    for i, img in enumerate(imgs):
        if i < n_train:
            dst = target_root / "train" / cls.name
        elif i < n_val:
            dst = target_root / "val" / cls.name
        else:
            dst = target_root / "test" / cls.name
        dst.mkdir(parents=True, exist_ok=True)
        shutil.copy(img, dst)
