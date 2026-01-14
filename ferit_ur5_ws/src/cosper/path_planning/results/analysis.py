import pandas as pd
from itertools import product

# Load CSV file
df = pd.read_csv("/home/RVLuser/ferit_ur5_ws/src/cosper/path_planning/results/results_multi-c_our_handle.csv")

# Select only the boolean columns (first 5 in your case)
bool_cols = ["path_found", "traj_success", "contact_free", "door_opened"]
bool_df = df[bool_cols]

# Count occurrences of each combination
combo_counts = bool_df.value_counts().reset_index()
combo_counts.columns = bool_cols + ["count"]

# Print the result
print(combo_counts)

# Optional: also print combinations that never occurred
all_combinations = pd.DataFrame(list(product([False, True], repeat=len(bool_cols))), columns=bool_cols)
merged = all_combinations.merge(combo_counts, on=bool_cols, how="left").fillna(0).astype({ "count": int })
print("\nAll possible combinations (including those not in the file):")
print(merged)