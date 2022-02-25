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

        T_man = np.eye(4)
        if user_question("Manual registration?"):
            T_man = manual_registration(source.crop, target.crop)

        T_icp = ICP(source, target, T_man, voxel_size)
        source.transform(T_icp @ T_man)

        process.saveTs()


if __name__ == "__main__":
    main()
