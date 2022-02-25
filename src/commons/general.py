import os
import pye57
import numpy as np
import open3d as o3d


def user_question(msg):
    user_answer = input(msg + " [y/n]: ")
    while True:

        if user_answer == "y" or user_answer == "yes":
            return True
        if user_answer == "n" or user_answer == "no":
            return False

        user_answer = input(" - chose between (y)es or (n)o: ")


def user_input(msg):
    user_answer = input(msg + ": ")
    while True:

        if isfloat(user_answer):
            return float(user_answer)
        elif user_answer == "":
            return None

        user_answer = input(" - insert a floating value: ")


def isfloat(num):
    try:
        float(num)
        return True
    except ValueError:
        return False


def read_cloud_e57(path):
    _, name = os.path.split(path)
    if os.path.exists(path):
        print("Reading \"{}\"".format(name))
    else:
        print("File \"{}\" do not exist".format(name))
        raise FileNotFoundError

    e57 = pye57.E57(path)
    cloud = e57.read_scan_raw(0)

    return cloud


def get_points(cloud):
    x = np.array(cloud["cartesianX"])
    y = np.array(cloud["cartesianY"])
    z = np.array(cloud["cartesianZ"])

    points = np.zeros((x.shape[0], 3))
    points[:, 0] = x
    points[:, 1] = y
    points[:, 2] = z

    return points


def get_colors(cloud, RGB=True):
    if RGB:
        r = np.array(cloud["colorRed"])
        g = np.array(cloud["colorGreen"])
        b = np.array(cloud["colorBlue"])
    else:
        r = np.array(cloud["intensity"])
        r = (r - r.min()) / (r.max() - r.min()) * 255
        g = r
        b = r

    colors = np.zeros((r.shape[0], 3))
    colors[:, 0] = r
    colors[:, 1] = g
    colors[:, 2] = b
    colors /= 255

    return colors


def create_cloud(points, colors):
    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    cloud.colors = o3d.utility.Vector3dVector(colors)
    print(cloud)

    return cloud


def downsample_cloud(cloud, size=0.05):
    print("Downsample outliers ...")

    cloud_ds = cloud.voxel_down_sample(voxel_size=size)
    return cloud_ds


def filter_outliers(cloud):
    print("Filtering outliers ...")
    cloud, _ = cloud.remove_statistical_outlier(nb_neighbors=20,
                                                std_ratio=2.0)
    points = []
    colors = []
    for point, color in zip(cloud.points, cloud.colors):
        if -100 < point[0] < 100 and -100 < point[1] < 100:
            points.append(point)
            colors.append(color)

    cloud.points = o3d.utility.Vector3dVector(points)
    cloud.colors = o3d.utility.Vector3dVector(colors)

    return cloud


def create_path_file(path_cloud, folder_new):
    path_folder_old, name = os.path.split(path_cloud)
    path, folder_old = os.path.split(path_folder_old)
    name = name[:-4] + ".ply"
    path_folder_new = os.path.join(path, folder_new)
    path_file = os.path.join(path_folder_new, name)

    if not os.path.exists(path_folder_new):
        os.makedirs(path_folder_new)

    return path_file


def save_cloud(path_file, cloud, confirm=False):
    res = ""
    if confirm:
        res = input("Save the cloud (y/n)? ")

    if not confirm or (res == "y" or res == "yes"):
        print("Saving the cloud ...")
        o3d.io.write_point_cloud(path_file, cloud)
    else:
        print("Cloud not saved")


def crop_cloud(cloud: o3d.geometry.PointCloud, size_area=3, size_height=2, display_msg=False):
    if display_msg:
        print("Cropping the cloud ...")
    points = o3d.utility.Vector3dVector(
        get_points_cuboid(size_area, size_height))
    bnb = o3d.geometry.OrientedBoundingBox.create_from_points(points)
    return cloud.crop(bnb)


def get_points_cuboid(size_area=3, size_height=2):
    center = {'x': 0, 'y': 0, 'z': 0}
    return np.array([
        # Vertices Polygon1
        [center['x'] + (size_area / 2), center['y'] + (
            size_area / 2), center['z'] + size_height],  # face-topright
        [center['x'] - (size_area / 2), center['y'] + (
            size_area / 2), center['z'] + size_height],  # face-topleft
        [center['x'] - (size_area / 2), center['y'] - (
            size_area / 2), center['z'] + size_height],  # rear-topleft
        [center['x'] + (size_area / 2), center['y'] - (
            size_area / 2), center['z'] + size_height],  # rear-topright

        # Vertices Polygon 2
        [center['x'] + (size_area / 2), center['y'] + (
            size_area / 2), center['z'] - size_height],
        [center['x'] - (size_area / 2), center['y'] + (
            size_area / 2), center['z'] - size_height],
        [center['x'] - (size_area / 2), center['y'] - (
            size_area / 2), center['z'] - size_height],
        [center['x'] + (size_area / 2), center['y'] - (
            size_area / 2), center['z'] - size_height],
    ]).astype("float64")


def load_cloud(path_cloud, T=np.eye(4)):
    _, name = os.path.split(path_cloud)
    print("Loading \"{}\"".format(name))
    cloud = o3d.io.read_point_cloud(path_cloud)
    cloud.transform(T)

    return cloud
