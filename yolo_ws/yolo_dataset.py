"""
Yolo Dataset Creator for Classification Tasks

This module provides functionality to create and manage datasets for YOLO classification.
It allows adding images to specific classes with automatic directory structure creation.
"""

import os
import shutil
from pathlib import Path
from typing import Union, List, Optional
from PIL import Image
import hashlib


class YoloDatasetCreator:
    """Class for creating and managing YOLO classification datasets."""
    
    def __init__(self, dataset_root: str):
        """
        Initialize the YOLO dataset creator.
        
        Args:
            dataset_root: Root directory for the dataset (e.g., 'datasets/flowers')
        """
        self.dataset_root = Path(dataset_root)
        self.train_dir = self.dataset_root / 'train'
        self.val_dir = self.dataset_root / 'val'
        self.test_dir = self.dataset_root / 'test'
        
    def create_structure(self) -> None:
        """Create the standard YOLO classification directory structure."""
        # Create train, val, test directories for each class
        for split_dir in [self.train_dir, self.val_dir, self.test_dir]:
            split_dir.mkdir(parents=True, exist_ok=True)
    
    def add_image_to_class(
        self, 
        image_path: Union[str, Path], 
        class_name: str, 
        split: str = 'train',
        copy: bool = True,
        validate_image: bool = True
    ) -> bool:
        """
        Add an image to a specific class in the dataset.
        
        Args:
            image_path: Path to the source image file
            class_name: Name of the class (directory name)
            split: Dataset split ('train', 'val', or 'test')
            copy: If True, copy the file; if False, move it
            validate_image: If True, validate that the file is a valid image
            
        Returns:
            True if successful, False otherwise
        """
        # Validate split
        if split not in ['train', 'val', 'test']:
            raise ValueError(f"Invalid split: {split}. Must be 'train', 'val', or 'test'")
        
        # Get the appropriate directory
        target_dir = {
            'train': self.train_dir,
            'val': self.val_dir,
            'test': self.test_dir
        }[split]
        
        # Create class directory
        class_dir = target_dir / class_name
        class_dir.mkdir(parents=True, exist_ok=True)
        
        # Validate image if requested
        if validate_image:
            try:
                with Image.open(image_path) as img:
                    img.verify()
            except Exception as e:
                print(f"Invalid image file {image_path}: {e}")
                return False
        
        # Determine destination path
        source_path = Path(image_path)
        dest_path = class_dir / source_path.name
        
        # Handle filename conflicts
        if dest_path.exists():
            dest_path = self._get_unique_filename(dest_path)
        
        # Copy or move the file
        try:
            if copy:
                shutil.copy2(image_path, dest_path)
            else:
                shutil.move(image_path, dest_path)
            return True
        except Exception as e:
            print(f"Error copying/moving file: {e}")
            return False
    
    def add_images_to_class(
        self, 
        image_paths: List[Union[str, Path]], 
        class_name: str, 
        split: str = 'train',
        copy: bool = True
    ) -> dict:
        """
        Add multiple images to a class.
        
        Args:
            image_paths: List of paths to source images
            class_name: Name of the class
            split: Dataset split ('train', 'val', or 'test')
            copy: If True, copy files; if False, move them
            
        Returns:
            Dictionary with success/failure counts
        """
        results = {'success': 0, 'failed': 0, 'total': len(image_paths)}
        
        for image_path in image_paths:
            if self.add_image_to_class(image_path, class_name, split, copy):
                results['success'] += 1
            else:
                results['failed'] += 1
        
        return results
    
    def _get_unique_filename(self, dest_path: Path) -> Path:
        """Generate a unique filename if the destination already exists."""
        stem = dest_path.stem
        suffix = dest_path.suffix
        counter = 1
        
        while True:
            new_name = f"{stem}_{counter}{suffix}"
            new_path = dest_path.parent / new_name
            if not new_path.exists():
                return new_path
            counter += 1
    
    def get_class_counts(self, split: str = 'train') -> dict:
        """
        Get the number of images in each class for a given split.
        
        Args:
            split: Dataset split ('train', 'val', or 'test')
            
        Returns:
            Dictionary mapping class names to image counts
        """
        target_dir = {
            'train': self.train_dir,
            'val': self.val_dir,
            'test': self.test_dir
        }[split]
        
        counts = {}
        if target_dir.exists():
            for class_dir in target_dir.iterdir():
                if class_dir.is_dir():
                    counts[class_dir.name] = len(list(class_dir.glob('*')))
        return counts
    
    def get_total_counts(self) -> dict:
        """Get image counts for all splits."""
        return {
            'train': self.get_class_counts('train'),
            'val': self.get_class_counts('val'),
            'test': self.get_class_counts('test')
        }
    
    def create_dataset_yaml(self, output_path: Optional[str] = None) -> str:
        """
        Create a dataset YAML file for YOLO training.
        
        Args:
            output_path: Path to save the YAML file. If None, saves to dataset root.
            
        Returns:
            Path to the created YAML file
        """
        if output_path is None:
            output_path = self.dataset_root / 'data.yaml'
        else:
            output_path = Path(output_path)
        
        # Get class names from train directory
        classes = sorted([d.name for d in self.train_dir.iterdir() if d.is_dir()])
        
        yaml_content = f"""# YOLO Classification Dataset
path: {self.dataset_root.absolute()}  # dataset root directory

# Classes
names:
{chr(10).join(f"  {i}: {cls}" for i, cls in enumerate(classes))}
"""
        
        output_path.write_text(yaml_content)
        return str(output_path)
    
    def split_dataset(
        self, 
        source_dir: Union[str, Path], 
        class_name: str,
        train_ratio: float = 0.8,
        val_ratio: float = 0.1,
        copy: bool = True
    ) -> dict:
        """
        Split images from a source directory into train/val/test sets.
        
        Args:
            source_dir: Directory containing source images
            class_name: Name of the class
            train_ratio: Proportion for training set
            val_ratio: Proportion for validation set
            copy: If True, copy files; if False, move them
            
        Returns:
            Dictionary with split statistics
        """
        source_dir = Path(source_dir)
        images = list(source_dir.glob('*'))
        images = [img for img in images if img.is_file()]
        
        # Shuffle and split
        import random
        random.shuffle(images)
        
        n = len(images)
        train_end = int(n * train_ratio)
        val_end = train_end + int(n * val_ratio)
        
        train_images = images[:train_end]
        val_images = images[train_end:val_end]
        test_images = images[val_end:]
        
        results = {
            'train': len(train_images),
            'val': len(val_images),
            'test': len(test_images)
        }
        
        # Add images to respective splits
        for img in train_images:
            self.add_image_to_class(img, class_name, 'train', copy)
        for img in val_images:
            self.add_image_to_class(img, class_name, 'val', copy)
        for img in test_images:
            self.add_image_to_class(img, class_name, 'test', copy)
        
        return results


# Convenience functions
def create_dataset(dataset_root: str) -> YoloDatasetCreator:
    """Create a new YOLO dataset creator instance."""
    return YoloDatasetCreator(dataset_root)


def add_image(image_path: Union[str, Path], class_name: str, dataset_root: str, 
              split: str = 'train', copy: bool = True) -> bool:
    """Convenience function to add a single image to a dataset."""
    creator = YoloDatasetCreator(dataset_root)
    return creator.add_image_to_class(image_path, class_name, split, copy)


if __name__ == '__main__':
    # Example usage
    print("YOLO Dataset Creator - Example Usage")
    print("=" * 50)
    
    # Create dataset
    dataset = YoloDatasetCreator('datasets/flowers')
    dataset.create_structure()
    print(f"Created dataset structure at: {dataset.dataset_root}")
    
    # Add some sample images (would use real paths in practice)
    print("\nTo add images, use:")
    print("  dataset.add_image_to_class('path/to/image.jpg', 'daisy', 'train')")
    print("  dataset.add_image_to_class('path/to/image.jpg', 'rose', 'val')")
    
    # Show structure
    print(f"\nDataset structure:")
    print(f"  Train: {dataset.train_dir}")
    print(f"  Val: {dataset.val_dir}")
    print(f"  Test: {dataset.test_dir}")
