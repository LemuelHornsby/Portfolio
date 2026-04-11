# --- S3 + CSV (hardcoded credentials) ---
import csv, boto3
from pathlib import Path
from datetime import datetime

BUCKET_NAME = "bbdgtwin"
s3 = boto3.client(
    "s3",
    aws_access_key_id="AKIAZZZS2DNDHROGADJZ",
    aws_secret_access_key="JApEs0Q4ckgJ6uaUlPvO4a8q3xu1yB8D+gLU7O9x",
    # region_name="us-east-1",  # uncomment if your bucket is in a specific region and uploads redirect
)

DATA_DIR = Path("pump_data"); DATA_DIR.mkdir(exist_ok=True)
CSV_HEADER = ["timestamp_iso","t_epoch","rpm","flow_meas","flow_sim","dp_meas","dp_sim"]
_active_minute_id = None
_minute_rows = []

def _minute_filename(minute_id: int) -> Path:
    ts = datetime.fromtimestamp(minute_id * 60)
    return DATA_DIR / f"pump_{ts:%Y%m%d_%H%M}.csv"

def _flush_minute_to_csv_and_upload():
    global _minute_rows, _active_minute_id
    if not _minute_rows or _active_minute_id is None:
        return
    fn = _minute_filename(_active_minute_id)
    new_file = not fn.exists()
    with fn.open("a", newline="") as f:
        w = csv.writer(f)
        if new_file:
            w.writerow(CSV_HEADER)
        w.writerows(_minute_rows)
    _minute_rows = []
    try:
        s3.upload_file(str(fn), BUCKET_NAME, fn.name)
        print(f"[S3] Uploaded {fn.name} → s3://{BUCKET_NAME}/{fn.name}")
    except Exception as e:
        print(f"[S3] Upload failed: {e}")
