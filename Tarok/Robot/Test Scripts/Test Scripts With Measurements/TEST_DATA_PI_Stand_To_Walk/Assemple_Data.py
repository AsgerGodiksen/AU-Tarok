"""
Assemble_Data.py

Reads the 30 Data_Analysis_For_Test_XX_*.txt files from
TEST_DATA_PI_Stand_To_Walk/Plots_For_Tests_XX/ and assembles a single
CSV with 30 rows × 48 columns.

Columns (4 metrics × 12 actuators = 48):
  {Leg}_{Joint}_Stand_RMSE_deg
  {Leg}_{Joint}_Swing_RMSE_deg
  {Leg}_{Joint}_Stand_AbsMean_Nm
  {Leg}_{Joint}_Swing_AbsMean_Nm

for Leg in [FL, FR, HL, HR] and Joint in [J1, J2, J3].

Usage:
  python3 Assemble_Data.py                      # auto-find in current dir
  python3 Assemble_Data.py /path/to/base/dir    # specify base directory
"""

import os
import sys
import re
import glob
import pandas as pd

# ─────────────────────────────────────────────────────────────────────────────
# Configuration
# ─────────────────────────────────────────────────────────────────────────────
LEGS   = ["FL", "FR", "HL", "HR"]
JOINTS = ["J1", "J2", "J3"]

BASE_DIR = sys.argv[1] if len(sys.argv) > 1 else os.path.dirname(os.path.abspath(__file__))


# ─────────────────────────────────────────────────────────────────────────────
# Parsing one report file
# ─────────────────────────────────────────────────────────────────────────────
def parse_report(txt_path: str) -> dict:
    """
    Parse a Data_Analysis_For_Test_XX_*.txt file and return a flat dict
    with 48 key-value pairs (4 metrics × 12 actuators).
    """
    with open(txt_path, "r") as f:
        content = f.read()

    row = {}

    # ── Extract Stand/Swing RMSE from the position-error table ───────────
    # Lines look like:  FL    J1              0.0723          0.0550      4730       593
    rmse_pattern = re.compile(
        r"^(FL|FR|HL|HR)\s+(J[123])\s+"       # leg  joint
        r"([\d.]+)\s+"                         # stand RMSE
        r"([\d.]+)\s+"                         # swing RMSE
        r"\d+\s+\d+",                          # stand N, swing N (ignored)
        re.MULTILINE,
    )
    for m in rmse_pattern.finditer(content):
        leg, jnt = m.group(1), m.group(2)
        row[f"{leg}_{jnt}_Stand_RMSE_deg"] = float(m.group(3))
        row[f"{leg}_{jnt}_Swing_RMSE_deg"] = float(m.group(4))

    # ── Extract Stand/Swing |mean| torque from the torque table ──────────
    # Lines look like:  FL    J1    0.9800  1.0422    0.9800    1.0422
    # Columns: signed stand, signed swing, |stand|, |swing|
    torque_pattern = re.compile(
        r"^(FL|FR|HL|HR)\s+(J[123])\s+"
        r"[-\d.]+\s+"                          # signed stand mean (skip)
        r"[-\d.]+\s+"                          # signed swing mean (skip)
        r"([\d.]+)\s+"                         # stand |mean|
        r"([\d.]+)",                           # swing |mean|
        re.MULTILINE,
    )
    for m in torque_pattern.finditer(content):
        leg, jnt = m.group(1), m.group(2)
        row[f"{leg}_{jnt}_Stand_AbsMean_Nm"] = float(m.group(3))
        row[f"{leg}_{jnt}_Swing_AbsMean_Nm"] = float(m.group(4))

    return row


# ─────────────────────────────────────────────────────────────────────────────
# Discover all report files
# ─────────────────────────────────────────────────────────────────────────────
pattern = os.path.join(BASE_DIR, "TEST_DATA_PI_Stand_To_Walk",
                       "Plots_For_Tests_*", "Data_Analysis_For_Test_*.txt")
txt_files = sorted(glob.glob(pattern))

if not txt_files:
    print(f"[ERROR] No report files found matching:\n  {pattern}")
    sys.exit(1)

print(f"Found {len(txt_files)} report file(s).\n")

# ─────────────────────────────────────────────────────────────────────────────
# Build the assembled table
# ─────────────────────────────────────────────────────────────────────────────
# Define column order: group by metric, then all 12 actuators
# Order: Stand RMSE (12) → Swing RMSE (12) → Stand |mean| Nm (12) → Swing |mean| Nm (12)
METRICS = ["Stand_RMSE_deg", "Swing_RMSE_deg", "Stand_AbsMean_Nm", "Swing_AbsMean_Nm"]
columns = []
for metric in METRICS:
    for leg in LEGS:
        for jnt in JOINTS:
            columns.append(f"{leg}_{jnt}_{metric}")

rows = []
test_ids = []

for txt_path in txt_files:
    # Extract test number from folder or filename
    m = re.search(r"Test[_\s]*(\d+)", txt_path)
    test_num = int(m.group(1)) if m else 0

    print(f"  Parsing Test {test_num:02d}: {os.path.basename(txt_path)}")
    row = parse_report(txt_path)

    # Verify we got all 48 values
    missing = [c for c in columns if c not in row]
    if missing:
        print(f"    [WARN] Missing {len(missing)} columns: {missing[:5]}...")

    rows.append(row)
    test_ids.append(test_num)

df = pd.DataFrame(rows, columns=columns)
df.insert(0, "Test", test_ids)

# ─────────────────────────────────────────────────────────────────────────────
# Save
# ─────────────────────────────────────────────────────────────────────────────
out_path = os.path.join(BASE_DIR, "Assembled_Data.csv")
df.to_csv(out_path, index=False, float_format="%.4f")

print(f"\nSaved: {out_path}")
print(f"  Shape: {df.shape[0]} rows × {df.shape[1]} columns "
      f"(1 Test ID + {len(columns)} data columns)")
print("Done.")