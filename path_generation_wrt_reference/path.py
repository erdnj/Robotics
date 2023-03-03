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


def transform_sd2cart(sd_path_2d, reference_path_2d):
    last_forwarded = 0
    forwarded_s = 0
    ref_it = iter(reference_path_2d)
    old_point = next(ref_it)
    new_point = next(ref_it)
    point_dif = new_point - old_point
    last_forwarded = np.linalg.norm(new_point - old_point)
    forwarded_s += last_forwarded
    transformed_points = []
    for s, d in sd_path_2d:
        while forwarded_s < s:
            old_point = new_point
            new_point = next(ref_it, None)
            if new_point is None:
                break
            point_dif = new_point - old_point
            last_forwarded = np.linalg.norm(new_point - old_point)
            forwarded_s += last_forwarded
        if new_point is None:
            break
        needed_forward = last_forwarded - (forwarded_s - s)
        target_point = old_point + point_dif * (needed_forward / last_forwarded)
        if d != 0:
            ort_point_diff = np.array([-point_dif[1], point_dif[0]])
            target_point += ort_point_diff * (d / last_forwarded)
        transformed_points.append(target_point)
    return np.array(transformed_points)


def plotOnReference(target_path, ref_path, color="red", with_dots=False, show=False):
    plt.plot(ref_path[:, 0], ref_path[:, 1], color='green')
    plt.plot(target_path[:, 0], target_path[:, 1], color=color)

    if with_dots:
        plt.scatter(target_path[:, 0], target_path[:, 1], color=color)

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

# plot one of them
plotOnReference(transformed_path1, xy_path, with_dots=True, show=True)

# plot many
# plotOnReference(transformed_path1, xy_path)
plotOnReference(transformed_path2, xy_path, color='blue')
plotOnReference(transformed_path3, xy_path, color='orange', show=True)
