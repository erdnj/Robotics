import random
import osmnx as ox
import osmnx.utils_graph
import osmnx.plot
import shapely
import numpy as np
from matplotlib import pyplot as plt


def plot_with_annotations(G):
    fig, ax = ox.plot_graph(G, show=False, close=False)
    for nodeId, spec in ox.graph_to_gdfs(G, edges=False).fillna('').iterrows():
        text = str(nodeId)
        ax.annotate(text, (spec.x, spec.y), c='y')
    plt.show()


def random_sd_path_gen(range_s: float, maximum_d, num_of_points):
    step_s = range_s / num_of_points
    return [
            (
                random.uniform(i * step_s, (i + 1) * step_s),
                random.uniform(-maximum_d, maximum_d)
            )
            for i in range(num_of_points)]


def transform_sd2cart(sd_path_2d, reference_path_2d):
    # movement between last iterated dot to previous one
    last_forwarded = 0

    # movement between start dot to last iterated one
    forwarded_s = 0

    # forward related
    ref_it = iter(reference_path_2d)
    old_point = next(ref_it)
    new_point = next(ref_it)
    point_dif = new_point - old_point
    last_forwarded = np.linalg.norm(new_point - old_point)
    forwarded_s += last_forwarded

    # ort unit vector related
    last_d = None
    prev_unit_ort_vector = None
    ort_point_diff = np.array([-point_dif[1], point_dif[0]])
    current_unit_ort_vector = ort_point_diff / last_forwarded

    transformed_points = []
    for s, d in sd_path_2d:
        while forwarded_s < s:
            old_point = new_point
            new_point = next(ref_it, None)
            if new_point is None:
                break
            # s related computations
            point_dif = new_point - old_point
            last_forwarded = np.linalg.norm(new_point - old_point)
            forwarded_s += last_forwarded

            # d related calculations
            prev_unit_ort_vector = current_unit_ort_vector
            ort_point_diff = np.array([-point_dif[1], point_dif[0]])
            current_unit_ort_vector = ort_point_diff / last_forwarded

            if last_d is None:
                continue
            # extra dots on reference dots
            extra_point = old_point.copy()
            if last_d != 0:
                extra_vector = current_unit_ort_vector + prev_unit_ort_vector
                extra_unit_vector = extra_vector / np.linalg.norm(extra_vector)
                extra_point += extra_unit_vector * last_d
            transformed_points.append(extra_point)

        if new_point is None:
            break
        last_d = d
        needed_forward = last_forwarded - (forwarded_s - s)
        target_point = old_point + point_dif * (needed_forward / last_forwarded)
        if d != 0:
            ort_point_diff = np.array([-point_dif[1], point_dif[0]])
            current_unit_ort_vector = ort_point_diff / last_forwarded
            target_point += d * current_unit_ort_vector

        transformed_points.append(target_point)
    return np.array(transformed_points)


# Func to plot given sd path on reference path
def plotOnReference(target_path, ref_path, color="red", with_dots=False, show=False):
    plt.plot(ref_path[:, 0], ref_path[:, 1], color='green')
    plt.plot(target_path[:, 0], target_path[:, 1], color=color)

    if with_dots:
        plt.scatter(target_path[:, 0], target_path[:, 1], color=color, s=4)

    plt.xlim(right=500, left=-500)
    plt.ylim(top=500, bottom=-500)
    plt.xlabel('x-axis')
    plt.ylabel('y-axis')
    if show:
        plt.show()


G = ox.graph_from_place(
    "Boğaziçi Üniversitesi Güney Yerleşkesi",
    network_type='drive',

)
route = [538400209, 269419454, 269419457]  # OSM node ids for South Campus ramp
sd_path_1 = [(0, 0), (20, 0), (70, 25), (300, 25)]
sd_path_2 = [(0, 0), (100, 0), (170, 25), (300, 25)]
sd_path_3 = [(0, 0), (200, 0), (270, 25), (300, 25)]
sd_path_random = random_sd_path_gen(range_s=500, maximum_d=50, num_of_points=8)
print(sd_path_random)
# Visualize the whole network
ox.plot_graph(G)

# Visualize the whole network with node ids
# plot_with_annotations(G)

# Visualize the route over the network
ox.plot.plot_graph_route(G, route)

# osmnx uses shapely library to represent edge.geometry
# We need to extract raw coordinates of the road
edges = ox.utils_graph.get_route_edge_attributes(G, route)
mls = shapely.MultiLineString([e['geometry'] for e in edges])
path = shapely.line_merge(mls)

xc, yc = path.xy  # in degree coordinates

# Conversion from longitude-latitude degree coordinates to meters
# ---
# The distance between two latitudes is always 111120m.
# The distance between two meridians is a function of latitude.
# Normalized to the starting point (South Campus Gate).
# ---
x = (np.array(xc) * np.cos(np.deg2rad(yc)) - xc[0] * np.cos(np.deg2rad(yc[0]))) * 111120
y = (np.array(yc) - yc[0]) * 111120
xy_path = np.column_stack((x, y))

# transform paths
transformed_path1 = transform_sd2cart(sd_path_1, xy_path)
transformed_path2 = transform_sd2cart(sd_path_2, xy_path)
transformed_path3 = transform_sd2cart(sd_path_3, xy_path)
transformed_random_path = transform_sd2cart(sd_path_random, xy_path)

# plot one of them
plotOnReference(transformed_path1, xy_path, show=True)
plotOnReference(transformed_path2, xy_path, color='blue', with_dots=True, show=True)
plotOnReference(transformed_path3, xy_path, color='orange', show=True)
plotOnReference(transformed_random_path, xy_path, color='blue', show=True)
plotOnReference(transformed_random_path, xy_path, color='red', show=True)
plotOnReference(transformed_random_path, xy_path, color='orange', show=True)
plotOnReference(transformed_random_path, xy_path, color='blue', show=True)

# plot many
# plotOnReference(transformed_path1, xy_path)
plotOnReference(transformed_path2, xy_path, color='blue')
plotOnReference(transformed_path3, xy_path, color='orange', show=True)
