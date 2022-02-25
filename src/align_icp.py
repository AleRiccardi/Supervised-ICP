import click

from commons.align import *


@ click.command()
@ click.argument(
    "path",
    type=str
)
@ click.option(
    "--voxel_size",
    default=0.1,
    type=float
)
def main(path, voxel_size):
    process = Process(path, "T_init.txt", "T_final.txt", voxel_size)

    while process.toContinue():
        source, target = process.getNext()
        T_icp = ICP(source, target, voxel_size=voxel_size)
        source.transform(T_icp)
        process.saveTs()


if __name__ == "__main__":
    main()
