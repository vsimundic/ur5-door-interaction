import pandas as pd

df = pd.read_csv('/home/RVLuser/data/multi-contact_results_single_contact_handle_moveit.csv')


duplicates = df[df.duplicated(keep=False)]
duplicate_indices = duplicates.index.tolist()
print("Duplicate rows:")
print(duplicates)
print("\nIndices of duplicate rows:")
print(duplicate_indices)