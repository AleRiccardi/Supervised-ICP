# Supervised ICP

A human supervised ICP software that gives you the tools to best align Point Clouds.  

## Requirements

The scripts require that you have a folder with `.ply.` point clouds to align.

## Usage

The script `align_general.py` is capable of performing manual and automatic 
alignment:

```python
python align_general.py /path_to_clouds/ --voxel_size=0.03
```

It is capable of reading initial guesses/transformations from the file "T\_init.txt":
```bash
Found 13 initial transformations, apply to the clouds? [y/n]: y
```

It is also capable of resuming from previous alignments processes saved in the file "T\_final.txt":
```bash
Found 3 final transformations, continue from there? [y/n]: y
```

Once the clouds begin to be loaded, then you can chose if you want to perform a manual registration where it will be asked to pick corresponding points:
```bash
Manual registration? [y/n]: y
```

Finally, you will have to specify the parameters for the ICP process.
Note that you should always use RANSAC when you do not provide an initial guess.
The combine target question asks you if you want to combine the last 3 clouds 
and provided it as a target cloud.

```bash
ICP parameters:
 - voxel size (0.03):
 - num interations (500000):
 - use ransac? [y/n]: n
 - combined target? [y/n]: y
```

## License
[MIT](https://choosealicense.com/licenses/mit/)
