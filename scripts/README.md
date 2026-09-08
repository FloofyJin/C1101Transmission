# PointMaker

This script generates file containing vector coordinate of the outline of an input image.

Input:
- .png or .mp4

Output:
- strands.json

## Command

```bash
python .\pointMaker.py .\rebel.png -o out --budget 90
# or
python .\pointMaker.py badapple.mp4 -o out --budget 90 --fps 20
```