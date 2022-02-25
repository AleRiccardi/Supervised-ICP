import click

from commons.align import *


@ click.command()
@ click.argument(
    "path",
    type=str
)
def main(path):
    process = Process(path, T_init="", T_final="T_init.txt", discard_prev=True)

    while process.toContinue():
        source, target = process.getNext()

        T_man = manual_registration(source.crop, target.crop)
        source.transform(T_man)

        process.saveTs()


if __name__ == "__main__":
    main()
