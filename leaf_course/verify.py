"""Default acceptance now uses the heap and whole-box workflow."""
import runpy
from pathlib import Path
runpy.run_path(str(Path(__file__).with_name("verify_heap.py")), run_name="__main__")
