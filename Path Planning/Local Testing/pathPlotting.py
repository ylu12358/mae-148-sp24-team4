import numpy as np
import pandas as pd
import matplotlib.pyplot as plt
from functions import rotateOrigin

# Load the CSV files
file2 = 'CSV/sortbot_donkey.txt'
file1 = 'CSV/sortbot_generated.txt'

# Read the CSV files into pandas DataFrames
df1 = pd.read_csv(file1, header=None, names=['Col1', 'Col2', 'Col3'])
df2 = pd.read_csv(file2, header=None, names=['Col1', 'Col2', 'Col3'])

# Display the first few rows of the DataFrames to understand their structure
print(df1.head())
print(df2.head())

# Cheese
coords = np.transpose(np.vstack((df1['Col1'], df1['Col2'], df1['Col3'])))
tilted = rotateOrigin(coords, 12)

# Assuming both CSV files have a common column to plot, for example, 'Date' and 'Value'
# Replace 'Date' and 'Value' with the actual column names in your CSV files

# Plotting data from the first CSV file
plt.figure(figsize=(10, 9))
plt.plot(tilted[:, 0] , tilted[:, 1], label='generated')

# Plotting data from the second CSV file
plt.plot(df2['Col1'], df2['Col2'], label='recorded')

# Adding titles and labels
ax = plt.gca()
ax.set_aspect('equal', adjustable='box')
plt.grid(True)
ax.set_title('Generated vs Recorded Path')
ax.set_xlabel('X')
ax.set_ylabel('Y')
plt.legend()

# Show the plot
plt.show()
