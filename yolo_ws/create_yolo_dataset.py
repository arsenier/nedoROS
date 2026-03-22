#!/usr/bin/env python3
"""
Script to create a YOLOv8 dataset from classification.csv file.
- Fixes image paths (replaces /data/upload/3/ with correct path)
- Creates YOLO format dataset with images organized by class
"""

import pandas as pd
import os
import shutil
from pathlib import Path

# Configuration
CSV_FILE = "clasification.csv"
# Old path from CSV file
ORIGINAL_IMAGE_BASE_PATH = "/data/upload/3"
# New path to actual images
NEW_IMAGE_BASE_PATH = "/Users/bw/Library/Application Support/label-studio/media/upload/3"

# Output dataset structure (matching dataset_ducks format)
OUTPUT_DATASET_DIR = "classification_dataset"


def fix_image_path(old_path: str, old_base: str, new_base: str) -> str:
    """Fix the image path by replacing the base directory."""
    # Extract relative path from old base
    if old_path.startswith(old_base):
        relative_path = old_path[len(old_base):].lstrip('/')
        new_path = os.path.join(new_base, relative_path)
        return new_path
    return old_path


def create_yolo_dataset():
    """Main function to create YOLO dataset from CSV."""
    
    # Read CSV file
    print(f"Reading CSV file: {CSV_FILE}")
    df = pd.read_csv(CSV_FILE)
    
    # Get unique classes from 'choice' column
    classes = df['choice'].dropna().unique().tolist()
    classes.sort()  # Sort for consistent ordering
    print(f"Found {len(classes)} classes: {classes}")
    
    # Create class to index mapping
    class_to_idx = {cls: idx for idx, cls in enumerate(classes)}
    print(f"Class to index mapping: {class_to_idx}")
    
    # Create output directories with class subdirectories (matching dataset_ducks structure)
    for split in ['train', 'val', 'test']:
        for cls in classes:
            os.makedirs(os.path.join(OUTPUT_DATASET_DIR, split, cls), exist_ok=True)
    
    # Process each row and copy images
    print("\nProcessing images...")
    processed_data = []
    
    for idx, row in df.iterrows():
        old_image_path = row['image']
        choice = row['choice']
        
        # Skip rows with empty choice
        if pd.isna(choice) or choice.strip() == '':
            print(f"Skipping row {idx}: empty choice")
            continue
        
        # Fix image path
        new_image_path = fix_image_path(old_image_path, ORIGINAL_IMAGE_BASE_PATH, NEW_IMAGE_BASE_PATH)
        
        # Check if image exists
        if not os.path.exists(new_image_path):
            print(f"Warning: Image not found: {new_image_path}")
            continue
        
        # Get filename
        filename = os.path.basename(new_image_path)
        
        # Determine split (use simple split by filename hash for reproducibility)
        file_hash = hash(filename)
        if file_hash % 10 < 8:
            split = 'train'
        elif file_hash % 10 < 9:
            split = 'val'
        else:
            split = 'test'
        
        # Copy image to output directory (organized by class)
        dest_image_path = os.path.join(OUTPUT_DATASET_DIR, split, choice, filename)
        if not os.path.exists(dest_image_path):
            shutil.copy2(new_image_path, dest_image_path)
        
        processed_data.append({
            'filename': filename,
            'split': split,
            'class': choice
        })
        
        if idx % 50 == 0:
            print(f"Processed {idx}/{len(df)} rows")
    
    print(f"\nTotal processed: {len(processed_data)} images")
    
    # Create data.yaml file (matching dataset_ducks format)
    data_yaml_content = f"""# YOLO Classification Dataset
path: {os.path.abspath(OUTPUT_DATASET_DIR)}  # dataset root directory

# Classes
names:
{chr(10).join(f'  {i}: {cls}' for i, cls in enumerate(classes))}
"""
    
    data_yaml_path = os.path.join(OUTPUT_DATASET_DIR, "data.yaml")
    with open(data_yaml_path, 'w') as f:
        f.write(data_yaml_content)
    
    print(f"\nCreated data.yaml at: {data_yaml_path}")
    print(f"Dataset structure:")
    print(f"  {OUTPUT_DATASET_DIR}/")
    print(f"    data.yaml")
    for split in ['train', 'val', 'test']:
        print(f"    {split}/")
        for cls in classes:
            count = sum(1 for d in processed_data if d['split'] == split and d['class'] == cls)
            print(f"      {cls}/  ({count} images)")
    
    # Print class distribution
    print(f"\nClass distribution:")
    for cls in classes:
        count = sum(1 for d in processed_data if d['class'] == cls)
        print(f"  {cls}: {count}")


if __name__ == "__main__":
    create_yolo_dataset()
