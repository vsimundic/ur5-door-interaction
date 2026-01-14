import os
import re
import pandas as pd
from collections import defaultdict
import yaml

# --- Load Configs ---
script_dir = os.path.dirname(os.path.abspath(__file__))
with open(os.path.join(script_dir, '../cfg/door_replanning_control.yaml'), 'r') as f:
    config = yaml.safe_load(f)

log_dir = config["rvl_log_dir"]
analysis_dir = config["analysis_dir"]

# --- Mapping of filenames to LaTeX-friendly labels ---
file_label_map = {
    "touch_success_baseline.log": r"Baseline",
    "touch_success_stdgxyk_0.02.log": r"$\sigma_{g,xy} = 0.02$",
    "touch_success_stdgxyk_0.1.log": r"$\sigma_{g,xy} = 0.1$",
    "touch_success_stdgzk_0.01.log": r"$\sigma_{g,z} = 0.01$",
    "touch_success_stdgzk_0.05.log": r"$\sigma_{g,z} = 0.05$",
    "touch_success_stdphi_1.0.log": r"$\sigma_{\varphi} = 1.0$",
    "touch_success_stdphi_3.0.log": r"$\sigma_{\varphi} = 3.0$",
    "touch_success_alpha_0.0001.log": r"$\alpha = 10^{-4}$",
    "touch_success_alpha_0.00001.log": r"$\alpha = 10^{-5}$",
    "touch_success_alpha_0.0000001.log": r"$\alpha = 10^{-7}$",
    "touch_success_nbest_0.log": r"$N_{best} = 0$",
    "touch_success_nbest_50.log": r"$N_{best} = 50$",
    "touch_success_nbest_100.log": r"$N_{best} = 100$",
    "touch_success_nbest_750.log": r"$N_{best} = 750$",
    "touch_success_vertexDistThr_0.05.log": r"$d_{vtx} = 0.05$",
    "touch_success_vertexDistThr_0.06.log": r"$d_{vtx} = 0.06$",
}

# --- Function to Parse Log Files for Success/Failure Counts ---
def parse_results(filepath):
    success, collision, miss = 0, 0, 0
    with open(filepath, 'r') as f:
        for line in f:
            match = re.match(r".+?,\s+attempts\s+\d+,\s+(\w+)", line.strip())
            if match:
                result = match.group(1).lower()
                if result == "success":
                    success += 1
                elif result == "collision":
                    collision += 1
                elif result == "miss":
                    miss += 1
    total = success + collision + miss
    success_rate = (success / total) * 100 if total > 0 else 0
    return success, collision, miss, success_rate

# --- Build Table Data ---
table_data = []
for filename, label in file_label_map.items():
    filepath = os.path.join(log_dir, filename)
    if not os.path.isfile(filepath):
        continue
    success, collision, miss, success_rate = parse_results(filepath)
    table_data.append({
        "Configuration": label,
        "Successes": success,
        "Collisions": collision,
        "Misses": miss,
        "Success Rate [\%]": f"{success_rate:.1f}"
    })

# --- Create and Display Table ---
df = pd.DataFrame(table_data)
df = df.sort_values(by="Success Rate [\%]", ascending=False).reset_index(drop=True)

# Save CSV if needed
df.to_csv(os.path.join(analysis_dir, "simulation_ablation_success_rates.csv"), index=False)

# Optionally print in LaTeX format
print(df.to_latex(index=False, escape=False, column_format='lcccc', float_format="%.1f"))