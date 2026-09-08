# animation stream

Streams animation data through UART. 

You must first populate the .json file with `pointMaker.py`.

```bash
python tools/animstream.py ..\..\scripts\out\strands.json
```

## Bake animation into build file

You have to toggle COMPILED_ANIMATION 1 in main.c for this to work.

Alternative option to stream data through UART is to build the data into the build file. This will generate animation_data.c and animation_data.h file. There is a limit to how big the animatio file can be since stm32's memory is limited to 64KB. I found 10 second videos at 90 strands/frame works fine.

You must first populate the .json file with `pointMaker.py`.

```bash
python .\tools\anim2c.py ..\..\scripts\out\strands.json --fps 20 --kbps 250 --preamble 16
```