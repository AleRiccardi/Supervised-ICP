import os
import click
import numpy as np
import open3d as o3d

from typing import Dict
from glob import glob

from commons import crop_cloud, load_cloud, user_question
from commons_align import display_clouds


def load_Ts(path):
    Ts: Dict[str, np.ndarray] = {}

    path_file = os.path.join(path)
    if not os.path.exists(path_file):
        return Ts

    file = open(path_file, "r")

    for line in file.readlines():
        line_split = line.split(",")
        path_cloud = line_split[0]
        T_str = line_split[1].lstrip()
        T_str = T_str.replace("[", "")
        T_str = T_str.replace("]", "")
        T_flt = np.fromstring(T_str, dtype=float, sep=' ')
        T = T_flt.reshape(4, -1)
        Ts[path_cloud] = T

    file.close()

    return Ts


def load_clouds(path_clouds, Ts):
    path_clouds_ply = glob(os.path.join(path_clouds, "*.ply"))
    path_clouds_ply = sorted(path_clouds_ply, key=lambda x: int(
        os.path.split(os .path.splitext(x)[0])[1]))

    clouds = []

    for path_cloud in path_clouds_ply:
        _, name = os.path.split(path_cloud)
        for path_T, T in Ts.items():
            if name in path_T:
                cloud = load_cloud(path_cloud)
                cloud = crop_cloud(cloud)
                cloud.transform(T)
                clouds.append(cloud)

    return clouds


@ click.command()
@ click.argument(
    "path",
    type=str
)
@ click.option(
    "--path_t",
    type=str
)
def main(path, path_t):
    Ts = load_Ts(path_t)
    clouds = load_clouds(path, Ts)

    if user_question("Display single clouds?"):
        for i in range(1, len(clouds)):
            display_clouds(clouds[i], clouds[i-1], color_source=True)

    if user_question("Display all clouds?"):
        o3d.visualization.draw_geometries(clouds)


if __name__ == "__main__":
    main()
