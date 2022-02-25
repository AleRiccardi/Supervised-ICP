import os
import time
import gtsam
import open3d as o3d
import numpy as np

from glob import glob
from copy import deepcopy
from typing import Dict, List

from commons.general import crop_cloud, user_input, user_question


class Cloud:
    CROP_W = 10
    CROP_H = 3

    def __init__(self, path="") -> None:
        if path and not os.path.exists(path):
            raise FileNotFoundError
        elif path:
            self.path = path

        self.loaded = False
        self.T = np.eye(4)
        self.createId()
        self.cloud = o3d.geometry.PointCloud()
        self.ds = o3d.geometry.PointCloud()
        self.crop = o3d.geometry.PointCloud()
        self.combined = o3d.geometry.PointCloud()

    def createId(self):
        _, name = os.path.split(self.path)
        self.id = int(name[:-4])

    def transform(self, T):
        if self.isLoaded():
            self.cloud.transform(T)
            self.crop.transform(T)
            self.ds.transform(T)
            self.combined.transform(T)

        # Update the transformation
        self.T = T @ self.T

    def load(self, voxel_size=0.1):
        if self.isLoaded():
            return

        print("Loading \"{}\"".format(self.id))

        self.cloud = o3d.io.read_point_cloud(self.path)
        self.ds = self.cloud.voxel_down_sample(voxel_size)
        self.crop = crop_cloud(self.cloud, self.CROP_W, self.CROP_H)
        self.combined = deepcopy(self.ds)

        self.cloud.transform(self.T)
        self.crop.transform(self.T)
        self.ds.transform(self.T)
        self.combined.transform(self.T)
        self.loaded = True

    def isLoaded(self):
        return self.loaded

    def combine(self, cloud):
        points1 = np.asarray(self.ds.points)
        points2 = np.asarray(cloud.ds.points)
        points3 = np.concatenate((points1, points2), axis=0)
        colors1 = np.asarray(self.ds.colors)
        colors2 = np.asarray(cloud.ds.colors)
        colors3 = np.concatenate((colors1, colors2), axis=0)

        self.combined.points = o3d.utility.Vector3dVector(points3)
        self.combined.colors = o3d.utility.Vector3dVector(colors3)

    def downsample(self, voxel_size):
        self.cloud.voxel_down_sample(voxel_size)
        self.cloud.voxel_down_sample(voxel_size)

    def free(self):
        self.cloud = o3d.geometry.PointCloud()
        self.ds = o3d.geometry.PointCloud()
        self.crop = o3d.geometry.PointCloud()
        self.combined = o3d.geometry.PointCloud()

    def __str__(self):
        T_str = str(self.T.flatten()).replace("\n", "")
        string = self.path + ", " + T_str + "\n"

        return string


class Process:
    NUM_CLOUD_LOADED = 3

    def __init__(self, path, T_init="T_init.txt", T_final="T_final.txt", voxel_size=0.3, discard_prev=False, reverse=False) -> None:
        if not os.path.exists(path):
            raise FileNotFoundError
        self.path = path
        self.T_init = T_init
        self.T_final = T_final
        self.voxel_size = voxel_size
        self.discard_prev = discard_prev
        self.reverse = reverse

        self.clouds: List[Cloud] = []

        self.Ts_init = self.loadTs(self.T_init)
        if len(self.Ts_init) > 0:
            choice_final = user_question(
                "Found {} initial transformations, apply to the clouds?".format(len(self.Ts_init)))

            if not choice_final:
                self.Ts_init: Dict[str, np.ndarray] = {}

        self.Ts_final = self.loadTs(self.T_final)
        if len(self.Ts_final) > 0:
            choice_final = user_question(
                "Found {} final transformations, continue from there?".format(len(self.Ts_final)))

            if not choice_final:
                self.Ts_final: Dict[str, np.ndarray] = {}

        self.loadClouds()

    def loadTs(self, T_file):
        Ts: Dict[str, np.ndarray] = {}
        if not T_file:
            return Ts

        path_file = os.path.join(self.path, T_file)
        if not os.path.exists(path_file):
            return Ts

        file = open(path_file, "r")
        for line in file.readlines():
            line_split = line.split(",")
            path_cloud = line_split[0]
            _, name = os.path.split(path_cloud)
            T_str = line_split[1].lstrip()
            T_str = T_str.replace("[", "")
            T_str = T_str.replace("]", "")
            T_flt = np.fromstring(T_str, dtype=float, sep=' ')
            T = T_flt.reshape(4, -1)
            Ts[name] = T
        file.close()

        return Ts

    def loadClouds(self):
        path_clouds = self.path
        path_clouds = glob(os.path.join(path_clouds, "*.ply"))
        path_clouds = sorted(path_clouds, reverse=self.reverse,
                             key=lambda x: int(os.path.split(os.path.splitext(x)[0])[1]))
        print("Found {} clouds".format(len(path_clouds)))

        self.idx_cur = 0
        for idx, path_cloud in enumerate(path_clouds):
            cloud = Cloud(path_cloud)

            _, name = os.path.split(path_cloud)
            if name in self.Ts_final:
                self.idx_cur = idx
                T = self.Ts_final[name]
                cloud.transform(T)

                if self.skipLoadingCloud():
                    print("Skip loading cloud \"{}\"".format(cloud.id))
                else:
                    cloud.load(self.voxel_size)
            elif name in self.Ts_init:
                T = self.Ts_init[name]
                cloud.transform(T)

            self.clouds.append(cloud)

    def skipLoadingCloud(self):
        if self.discard_prev and self.idx_cur < len(self.Ts_final):
            return True
        elif self.idx_cur + self.NUM_CLOUD_LOADED < len(self.Ts_final):
            return True

        return False

    def toContinue(self):
        if self.idx_cur < len(self.clouds)-1:
            return True

        return False

    def getCurrent(self):
        target = self.clouds[self.idx_cur - 1]
        source = self.clouds[self.idx_cur]
        target.load(self.voxel_size)
        source.load(self.voxel_size)

        return source, target

    def getNext(self):
        self.idx_cur += 1

        target = self.createTarget()

        source = self.clouds[self.idx_cur]
        source.load(self.voxel_size)

        self.freeClouds()

        return source, target

    def getNextSingle(self):
        cloud = self.clouds[self.idx_cur]
        cloud.load(self.voxel_size)
        self.idx_cur += 1

        return cloud

    def createTarget(self):
        target = self.clouds[self.idx_cur-1]
        target.load(self.voxel_size)

        for i in range(0, self.idx_cur-1):
            cloud = self.clouds[i]
            if not cloud.isLoaded():
                continue
            target.combine(cloud)

        target.downsample(self.voxel_size)
        return target

    def createDoubleTarget(self):
        target = self.clouds[self.idx_cur-1]
        target.load(self.voxel_size)
        if self.idx_cur > 2:
            cloud = self.clouds[self.idx_cur-2]
            cloud.load(self.voxel_size)
            target.combine(cloud)

        target.downsample(self.voxel_size)
        return target

    def getCloud(self, position):
        idx = self.idx_cur - position
        if idx < 0:
            raise IndexError

        return self.clouds[idx]

    def saveTs(self):
        path_file_tmp = os.path.join(self.path, self.T_final + ".tmp")
        path_file = os.path.join(self.path, self.T_final)

        file = open(path_file_tmp, "w")
        count = 0
        for idx, cloud in enumerate(self.clouds):
            # Important statement (if not correct will not save the progresses)
            if idx > self.idx_cur and not cloud.isLoaded():
                break

            file.write(str(cloud))
            count += 1
        file.close()
        print("Saved {} transformations".format(count))
        os.replace(path_file_tmp, path_file)

    def update(self, values: gtsam.Values):
        for cloud in self.clouds:
            if not values.exists(cloud.id):
                continue

            pose = values.atPose3(cloud.id)
            cloud.T = pose.matrix()

    def freeClouds(self):
        for i in range(0, self.idx_cur, -1):
            if self.idx_cur - i > self.NUM_CLOUD_LOADED:
                self.clouds[i].free()
                print("Freed cloud \"{}\"".format(self.cloud[i].id))


class Optimizer():
    def __init__(self) -> None:
        self.graph = gtsam.NonlinearFactorGraph()
        self.values = gtsam.Values()

        self.noise_init = gtsam.noiseModel.Diagonal.Sigmas(
            [0, 0, 0, 0, 0, 0])
        self.noise_inaccurate = gtsam.noiseModel.Diagonal.Sigmas(
            [7*np.pi/180, 7*np.pi/180, 7*np.pi/180, 0.25, 0.25, 0.25])
        self.noise_accurate = gtsam.noiseModel.Diagonal.Sigmas(
            [5*np.pi/180, 5*np.pi/180, 5*np.pi/180, 0.05, 0.05, 0.05])

    def initialize(self, process: Process):
        for idx, cloud in enumerate(process.clouds):
            if idx > process.idx_cur:
                break

            if idx == 0:
                self.addValue(cloud)
                self.addInit(cloud)
                continue

            self.addValue(cloud)
            self.addPrior(cloud.id, cloud.T, accurate=True)

    def optimize(self):
        optimizer = gtsam.LevenbergMarquardtOptimizer(
            self.graph, self.values)
        self.values = optimizer.optimize()

    def add(self, source, target, T_prior):
        self.addValue(source)
        self.addPrior(source.id, T_prior)
        self.addBetween(source, target)

    def addValue(self, source):
        pose_cur = gtsam.Pose3(source.T)
        self.values.insert(source.id, pose_cur)

    def addInit(self, source):
        pose_cur = gtsam.Pose3(source.T)
        self.graph.add(gtsam.PriorFactorPose3(
            source.id, pose_cur, self.noise_init))

    def addPrior(self, id_source, T, accurate=False):
        #  PriorFactorPose3
        noise = self.noise_accurate if accurate else self.noise_inaccurate
        pose = gtsam.Pose3(T)
        self.graph.add(gtsam.PriorFactorPose3(id_source, pose, noise))

    def addBetween(self, source: Cloud, target: Cloud, accurate=True):
        # BetweenFactorPose3
        pose_prev = gtsam.Pose3(target.T)
        pose_cur = gtsam.Pose3(source.T)
        odom = pose_prev.between(pose_cur)

        noise = self.noise_accurate if accurate else self.noise_inaccurate
        self.graph.add(gtsam.BetweenFactorPose3(
            target.id, source.id, odom, noise))

    def getValues(self):
        return self.values


def filter_outliers(cloud):
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


def pick_points(cloud):
    vis = o3d.visualization.VisualizerWithEditing()
    vis.create_window()
    vis.add_geometry(cloud)
    vis.run()  # user picks points
    vis.destroy_window()
    print("")
    return vis.get_picked_points()


def compute_features(cloud_ds, voxel_size):
    radius_normal = voxel_size * 2
    cloud_ds.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))

    radius_feature = voxel_size * 5
    cloud_fpfh = o3d.pipelines.registration.compute_fpfh_feature(
        cloud_ds,
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_feature, max_nn=100))

    return cloud_fpfh


def execute_global_registration(source_down, target_down, source_fpfh,
                                target_fpfh, voxel_size):
    distance_threshold = voxel_size * 1.5
    result = o3d.pipelines.registration.registration_ransac_based_on_feature_matching(
        source_down, target_down, source_fpfh, target_fpfh, True,
        distance_threshold,
        o3d.pipelines.registration.TransformationEstimationPointToPoint(False),
        3, [
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnEdgeLength(
                0.9),
            o3d.pipelines.registration.CorrespondenceCheckerBasedOnDistance(
                distance_threshold)
        ], o3d.pipelines.registration.RANSACConvergenceCriteria(30000, 0.999))
    return result


def refine_registration(source, target, initial_guess, voxel_size, num_iterations=50000):
    radius_normal = voxel_size
    source.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
    target.estimate_normals(
        o3d.geometry.KDTreeSearchParamHybrid(radius=radius_normal, max_nn=30))
    distance_threshold = voxel_size
    kernel_threshold = 0.3 * voxel_size
    robust_kernel = o3d.pipelines.registration.GMLoss(kernel_threshold)
    result = o3d.pipelines.registration.registration_icp(
        source, target, distance_threshold, initial_guess,
        o3d.pipelines.registration.TransformationEstimationPointToPlane(
            robust_kernel),
        o3d.pipelines.registration.ICPConvergenceCriteria(1e-06, 1e-06, int(num_iterations)))
    return result.transformation


def add_cloud(cloud, cloud_to_add):
    points1 = np.asarray(cloud.points)
    points2 = np.asarray(cloud_to_add.points)
    points3 = np.concatenate((points1, points2), axis=0)
    cloud.points = o3d.utility.Vector3dVector(points3)

    colors1 = np.asarray(cloud.colors)
    colors2 = np.asarray(cloud_to_add.colors)
    colors3 = np.concatenate((colors1, colors2), axis=0)
    cloud.colors = o3d.utility.Vector3dVector(colors3)


def manual_registration(source: o3d.geometry.PointCloud, target: o3d.geometry.PointCloud):
    T_man = np.eye(4)
    repeat = True
    while repeat:

        T_man = manual_registration_(source, target)
        display_clouds(source, target, T_man, color_source=True)

        if user_question("Transformation correct?"):
            repeat = False

    return T_man


def manual_registration_(source: o3d.geometry.PointCloud, target: o3d.geometry.PointCloud):

    picked_id_source = pick_points(source)
    picked_id_target = pick_points(target)

    if len(picked_id_source) != len(picked_id_target):
        print("You must pick the same number of points per cloud")
        return np.eye(4)
    if len(picked_id_source) < 3 or len(picked_id_target) < 3:
        print("You must pick at least 3 points per cloud")
        return np.eye(4)

    corr = np.zeros((len(picked_id_source), 2))
    corr[:, 0] = picked_id_source
    corr[:, 1] = picked_id_target

    p2p = o3d.pipelines.registration.TransformationEstimationPointToPoint()
    T = p2p.compute_transformation(source, target,
                                   o3d.utility.Vector2iVector(corr))

    return T


def ICP(source: Cloud, target: Cloud, T_init=np.eye(4), voxel_size=0.1, num_iterations=500000):
    T_icp = np.eye(4)
    repeat = True
    while repeat:
        vs_user, ni_user, ransac, combined = ICP_parameters(
            voxel_size, num_iterations)
        target_cloud = target.combined if combined else target.cloud
        T_icp = ICP_(source.cloud, target_cloud,
                     T_init, vs_user, ni_user, ransac=ransac)
        display_clouds(source.crop, target.crop,
                       T_icp @ T_init, color_source=True)

        if user_question("Transformation correct?"):
            repeat = False
        elif user_question("Execute manual registration?"):
            T_icp = manual_registration(source.crop, target.crop)
            if user_question("Manual transformation correct?"):
                repeat = False

    return T_icp


def ICP_(source: o3d.geometry.PointCloud, target: o3d.geometry.PointCloud, T_init=np.eye(4), voxel_size=0.1, num_iterations=500000, ransac=False):
    target_ds = target.voxel_down_sample(voxel_size)
    source_ds = deepcopy(source).voxel_down_sample(voxel_size)
    source_ds.transform(T_init)

    print("Executing ICP:")
    start = time.time()

    T_ransac = np.eye(4)

    if ransac:
        print(" - ransac ...")
        src_fpfh = compute_features(source_ds, voxel_size)
        trg_fpfh = compute_features(target_ds, voxel_size)
        ransac_result = execute_global_registration(
            source_ds, target_ds, src_fpfh, trg_fpfh, voxel_size)
        T_ransac = ransac_result.transformation

    print(" - refine ...")
    T_icp = refine_registration(
        source_ds, target_ds, T_ransac, voxel_size, num_iterations)
    end = time.time()
    print(" - time (sec): {}".format(round(end-start, 3)))

    return T_icp


def ICP_parameters(voxel_size, num_iterations):
    print("ICP parameters:")
    voxel_size_user = user_input(" - voxel size ({})".format(voxel_size))
    num_iterations_user = user_input(
        " - num interations ({})".format(num_iterations))
    ransac_user = user_question(" - use ransac?")
    combine_user = user_question(" - combined target?")

    voxel_size_user = voxel_size_user if voxel_size_user else voxel_size
    num_iterations_user = num_iterations_user if num_iterations_user else num_iterations

    return voxel_size_user, num_iterations_user, ransac_user, combine_user


def display_clouds(source: o3d.geometry.PointCloud,
                   target: o3d.geometry.PointCloud,
                   T=np.eye(4), color_source=False, color_target=False):
    source_temp = deepcopy(source)
    target_temp = deepcopy(target)
    source_temp.transform(T)

    if color_source:
        source_temp.paint_uniform_color([1, 0.706, 0])
    if color_target:
        target_temp.paint_uniform_color([0, 0.651, 0.929])

    o3d.visualization.draw_geometries([source_temp, target_temp])


def to_origin(cloud: o3d.geometry.PointCloud):

    T = np.eye(4)
    repeat = True
    while repeat:

        idxs = pick_points(cloud)

        if len(idxs) != 2:
            print("You must pick two points")
            continue

        position1 = cloud.points[idxs[0]]
        position2 = cloud.points[idxs[1]]
        position = (position1 + position2) / 2
        position[2] += 0.5
        T = np.eye(4)
        T[:3, 3] = position
        T = np.linalg.inv(T)

        cloud_tmp = deepcopy(cloud)
        cloud_tmp.transform(T)
        cloud_tmp = crop_cloud(cloud_tmp)

        display_clouds(cloud_tmp, o3d.geometry.PointCloud())

        if user_question("Transformation correct?"):
            repeat = False

    return T
