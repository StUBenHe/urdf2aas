import math
import yaml

def degrees_constructor(loader, node):
    value = loader.construct_scalar(node)
    return float(value) * math.pi / 180.0

yaml.SafeLoader.add_constructor('!degrees', degrees_constructor)

data = yaml.safe_load("""
shoulder_pan_joint:
  lower: !degrees -180
  upper: !degrees 180
""")

print(data)
