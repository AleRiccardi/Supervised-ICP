import click
import os
import open3d as o3d

from glob import glob

from commons.general import (crop_cloud, downsample_cloud, read_cloud_e57,
                             get_points, get_colors, create_cloud,
                             create_path_file, save_cloud, user_input)


def load_files(path_folder_in, folder_out):
    path_clouds = glob(os.path.join(path_folder_in, "*.e57"))
    path_clouds = sorted(path_clouds, key=lambda x: int(
        os.path.split(os.path.splitext(x)[0])[1]))

    path, folder_in = os.path.split(path_folder_in)
    path_folder_out = os.path.join(path, folder_out)

    path_clouds_ply = glob(os.path.join(path_folder_out, "*.ply"))
    path_clouds_ply = sorted(path_clouds_ply, key=lambda x: int(
        os.path.split(os.path.splitext(x)[0])[1]))

    if len(path_clouds_ply) > 0:
        choice_continue = user_input(
            "Found {} clouds already converted, continue from there?".format(len(path_clouds_ply)))

        if not choice_continue:
            return path_clouds

    # Remove duplicate clouds
    path_clouds_new = []
    for path_cloud in path_clouds:
        _, name_e57 = os.path.split(path_cloud)

        skip = False
        for path_cloud_ply in path_clouds_ply:
            _, name_ply = os.path.split(path_cloud_ply)
            if name_e57[:-4] == name_ply[:-4]:
                skip = True

        if not skip:
            path_clouds_new.append(path_cloud)

    return path_clouds_new


@ click.command()
@ click.argument(
    "path"
)
@ click.option(
    "--voxel_size",
    type=float,
    default=-1
)
@ click.option(
    "--restrict",
    default=False
)
@ click.option(
    "--display",
    default=False
)
def main(path, voxel_size, restrict, display):
    folder_out = "ply"

    if restrict:
        folder_out = "ply_restrict"
    elif voxel_size != -1:
        folder_out = "ply_ds"

    path_clouds = load_files(path, folder_out)

    for path_cloud in path_clouds:
        cloud_e57 = read_cloud_e57(path_cloud)

        points = get_points(cloud_e57)
        colors = get_colors(cloud_e57, RGB=True)
        cloud = create_cloud(points, colors)

        if voxel_size != -1:
            cloud = downsample_cloud(cloud, voxel_size)

        if restrict:
            cloud = crop_cloud(cloud, 4, 2)
        #  else:
            #  cloud = crop_cloud(cloud, 500, 10)

        if display:
            o3d.visualization.draw_geometries([cloud])

        path_file = create_path_file(path_cloud, folder_out)
        save_cloud(path_file, cloud)


if __name__ == "__main__":
    main()
