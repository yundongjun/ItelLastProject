
import os

def update_labels(directory):
    for filename in os.listdir(directory):
        if filename.endswith(".txt"):
            filepath = os.path.join(directory, filename)
            with open(filepath, 'r') as f:
                lines = f.readlines()
            
            new_lines = []
            for line in lines:
                parts = line.strip().split()
                class_id = int(parts[0])
                
                if class_id == 0:
                    parts[0] = '1'
                elif class_id == 1:
                    parts[0] = '2'
                
                new_lines.append(" ".join(parts))
            
            with open(filepath, 'w') as f:
                f.write("\n".join(new_lines))

if __name__ == "__main__":
    update_labels("train/labels")
    update_labels("valid/labels")
    print("Label files in train/labels and valid/labels have been updated.")
