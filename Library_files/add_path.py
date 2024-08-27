import os
import sys

parent_path = os.path.dirname(os.path.abspath(os.path.dirname(__file__)))
parent2_path = os.path.dirname(os.path.abspath(parent_path))
sys.path.append(parent_path)
sys.path.append(parent2_path)
if parent_path in sys.path and parent2_path in sys.path:
    pass
else:
    raise Exception('Parent path is not appended sys.path')