import numpy as np

# Load the .npy file
file_path = "tasks/zeyu_test/gpt-4o_V202412131811/raw_depth_image.npy"
data = np.load(file_path)
# Print the shape and data type
print(f"Shape: {data.shape}, Data Type: {data.dtype}")

# Check the first few elements (if 1D or 2D)
print(data[:5])  # Adjust slice for your use case
# Access the value at row 2, column 3
value = data[203, 159]
print(f"Value at (2, 3): {value}")
