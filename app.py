import os
import gzip
import math
import urllib.request
import osmnx as ox
import networkx as nx
from flask import Flask, request, jsonify

app = Flask(__name__)

GRAPHML_PATH = "sf_walk_network_elevation.graphml"
GRAPHML_GZ_URL = "https://github.com/LineupMahmood/flat-route-api/releases/download/v1.0/sf_walk_network_elevation.graphml.gz"

if not os.path.exists(GRAPHML_PATH):
    print("Graph file not found. Downloading...")
    gz_path = GRAPHML_PATH + ".gz"
    urllib.request.urlretrieve(GRAPHML_GZ_URL, gz_path)
    print("Download complete. Decompressing...")
    with gzip.open(gz_path, 'rb') as f_in:
        with open(GRAPHML_PATH, 'wb') as f_out:
            f_out.write(f_in.read())
    os.remove(gz_path)
    print("Decompression complete.")

import pickle

PICKLE_PATH = "sf_walk_network_v3.pkl"

print("Loading elevation network...")
if os.path.exists(PICKLE_PATH):
    print("Found pickle cache, loading fast...")
    with open(PICKLE_PATH, "rb") as f:
        G = pickle.load(f)
    print("Pickle loaded.")
else:
    print("No pickle found, loading from graphml (slow, one-time)...")
    G = ox.load_graphml(filepath=GRAPHML_PATH)
    for u, v, k, data in G.edges(keys=True, data=True):
        grade = float(data.get("grade_abs", 0))
        length = float(data.get("length", 0))
        data["impedance_high"] = length * (999999 if grade > 0.10 else (1 + 5000 * grade ** 2))
        data["impedance_max"]  = length * (999999 if grade > 0.07 else (1 + 15000 * grade ** 2))
    print("Saving pickle cache for fast future loads...")
    with open(PICKLE_PATH, "wb") as f:
        pickle.dump(G, f)

print("Network ready. Server starting...")


def analyze_route(route):
    total_gain = 0
    total_length = 0
    grades = []
    for i in range(len(route) - 1):
        u, v = route[i], route[i+1]
        edge_data = G.get_edge_data(u, v)
        edge = edge_data[0] if edge_data else {}
        length = float(edge.get("length") or 0)
        grade = float(edge.get("grade") or 0)
        grade_abs = abs(float(edge.get("grade_abs") or abs(grade)))
        if length * grade > 0:
            total_gain += length * grade
        total_length += length
        if length > 0:
            grades.append(grade_abs)

    max_grade = max(grades) if grades else 0
    avg_grade = sum(grades) / len(grades) if grades else 0
    coords = [{"lat": G.nodes[n]["y"], "lng": G.nodes[n]["x"]} for n in route]
    return {
        "coordinates": coords,
        "distanceInMiles": round(total_length / 1609.34, 2),
        "elevationGainFt": round(total_gain * 3.281, 1),
        "maxGradePct": round(max_grade * 100, 1),
        "avgGradePct": round(avg_grade * 100, 1),
        "_difficulty": avg_grade * 0.7 + max_grade * 0.3
    }

def has_backtrack(route, destination, threshold=0.25):
    """
    Returns True if the route backtracks significantly.
    At any point, if distance to destination increases by more than
    threshold * total_direct_distance, it's backtracking.
    """
    dest_lat = G.nodes[destination]["y"]
    dest_lng = G.nodes[destination]["x"]
    start_lat = G.nodes[route[0]]["y"]
    start_lng = G.nodes[route[0]]["x"]

    direct_dist = math.sqrt(
        ((dest_lat - start_lat) * 111000) ** 2 +
        ((dest_lng - start_lng) * 111000 * math.cos(math.radians(start_lat))) ** 2
    )
    max_allowed_increase = direct_dist * threshold

    prev_dist = direct_dist
    for node in route[1:]:
        nlat = G.nodes[node]["y"]
        nlng = G.nodes[node]["x"]
        dist = math.sqrt(
            ((dest_lat - nlat) * 111000) ** 2 +
            ((dest_lng - nlng) * 111000 * math.cos(math.radians(nlat))) ** 2
        )
        if dist > prev_dist + max_allowed_increase:
            return True
        prev_dist = dist
    return False

def get_route_via_waypoint(origin, destination, waypoint_node, weight):
    try:
        if waypoint_node in (origin, destination):
            return None
        leg1 = ox.routing.shortest_path(G, origin, waypoint_node, weight=weight)
        leg2 = ox.routing.shortest_path(G, waypoint_node, destination, weight=weight)
        if leg1 and leg2:
            return leg1 + leg2[1:]
    except:
        pass
    return None

def get_local_waypoint_nodes(origin, destination):
    """
    Generate waypoint candidates in a cross pattern at multiple radii.
    Wider offsets find flat detour corridors (like Octavia Blvd).
    """
    slat = G.nodes[origin]["y"]
    slng = G.nodes[origin]["x"]
    elat = G.nodes[destination]["y"]
    elng = G.nodes[destination]["x"]

    direct_dist_m = math.sqrt(
        ((elat - slat) * 111000) ** 2 +
        ((elng - slng) * 111000 * math.cos(math.radians(slat))) ** 2
    )

    mid_lat = (slat + elat) / 2
    mid_lng = (slng + elng) / 2

    candidate_coords = []
    # Try 3 radii: 30%, 60%, 100% of direct distance
    for factor in [0.3, 0.6, 1.0]:
        offset = max(direct_dist_m * factor, 200) / 111000
        candidate_coords += [
            (mid_lat + offset, mid_lng),
            (mid_lat - offset, mid_lng),
            (mid_lat, mid_lng + offset),
            (mid_lat, mid_lng - offset),
            (slat + offset, slng),
            (slat, slng + offset),
            (elat + offset, elng),
            (elat, elng + offset),
        ]

    nodes = []
    for lat, lng in candidate_coords:
        try:
            n = ox.distance.nearest_nodes(G, lng, lat)
            if n not in (origin, destination) and n not in nodes:
                nodes.append(n)
        except:
            pass

    # Sample flat nodes from the graph within an expanded bounding box
    padding = (direct_dist_m * 1.5) / 111000
    min_lat = min(slat, elat) - padding
    max_lat = max(slat, elat) + padding
    min_lng = min(slng, elng) - padding
    max_lng = max(slng, elng) + padding

    flat_candidates = []
    for node_id, data in G.nodes(data=True):
        nlat = data.get("y")
        nlng = data.get("x")
        if nlat is None or nlng is None:
            continue
        if not (min_lat <= nlat <= max_lat and min_lng <= nlng <= max_lng):
            continue
        if node_id in (origin, destination):
            continue
        # Score this node by average grade of its edges
        edge_grades = [
            float(d.get("grade_abs") or 0)
            for _, _, d in G.edges(node_id, data=True)
        ]
        if not edge_grades:
            continue
        avg_node_grade = sum(edge_grades) / len(edge_grades)
        flat_candidates.append((avg_node_grade, node_id))

    # Take the 20 flattest nodes as additional waypoint candidates
    flat_candidates.sort(key=lambda x: x[0])
    for _, node_id in flat_candidates[:20]:
        if node_id not in nodes:
            nodes.append(node_id)

    return nodes

def has_loop(route):
    """Detect routes that visit the same node twice — waypoint artifacts."""
    seen = set()
    for coord in route["coordinates"]:
        key = (round(coord["lat"], 5), round(coord["lng"], 5))
        if key in seen:
            return True
        seen.add(key)
    return False

def deduplicate_routes(routes):
    routes = [r for r in routes if not has_loop(r)]
    unique = []
    for r in routes:
        coords = r["coordinates"]
        if len(coords) < 2:
            continue
        is_dup = False
        for u in unique:
            # Same distance within 0.05mi AND same avg grade within 0.5% = duplicate
            same_dist = abs(r["distanceInMiles"] - u["distanceInMiles"]) < 0.05
            same_grade = abs(r["avgGradePct"] - u["avgGradePct"]) < 0.5
            if same_dist and same_grade:
                is_dup = True
                break
            # Also check midpoint proximity
            u_coords = u["coordinates"]
            mid = len(coords) // 2
            u_mid = len(u_coords) // 2
            if mid < len(coords) and u_mid < len(u_coords):
                dlat = coords[mid]["lat"] - u_coords[u_mid]["lat"]
                dlng = coords[mid]["lng"] - u_coords[u_mid]["lng"]
                dist_m = math.sqrt(dlat**2 + dlng**2) * 111000
                if dist_m < 40:
                    is_dup = True
                    break
        if not is_dup:
            unique.append(r)
    return unique

@app.route("/health", methods=["GET"])
def health():
    # Sample 5 edges to check grade data
    sample = []
    for u, v, data in list(G.edges(data=True))[:5]:
        sample.append({
            "grade_abs": data.get("grade_abs"),
            "impedance_high": data.get("impedance_high")
        })
    return {"status": "ok", "version": "v4-backtrack-filter", "sample_edges": sample}

@app.route("/route", methods=["GET"])
def get_route():
    try:
        if request.args.get("start") and request.args.get("end"):
            start_lat, start_lng = map(float, request.args.get("start").split(","))
            end_lat, end_lng = map(float, request.args.get("end").split(","))
        else:
            start_lat = float(request.args.get("start_lat"))
            start_lng = float(request.args.get("start_lng"))
            end_lat = float(request.args.get("end_lat"))
            end_lng = float(request.args.get("end_lng"))

        origin = ox.distance.nearest_nodes(G, start_lng, start_lat)
        destination = ox.distance.nearest_nodes(G, end_lng, end_lat)

        all_routes = []

        # Base routes
        for weight in ["impedance_high", "impedance_max", "length"]:
            r = ox.routing.shortest_path(G, origin, destination, weight=weight)
            if r:
                if has_backtrack(r, destination):
                    print(f"⚠️ Skipping backtracking {weight} route")
                    continue
                all_routes.append(analyze_route(r))

        unique_routes = deduplicate_routes(all_routes)
        unique_routes.sort(key=lambda r: r["_difficulty"])

        if not unique_routes:
            return jsonify({"error": "No routes found"}), 500

        min_dist = min(r["distanceInMiles"] for r in unique_routes)
        max_allowed = max(min_dist * 4.0, 1.5)
        filtered = [r for r in unique_routes
                    if r["distanceInMiles"] <= max_allowed
                    and r["maxGradePct"] <= 20.0]
        # Fall back if filters are too aggressive
        if not filtered:
            filtered = unique_routes

        for r in filtered:
            r.pop("_difficulty", None)

        flat = filtered[0]
        short = min(filtered, key=lambda r: r["distanceInMiles"])

        print(f"✅ Returning {len(filtered)} routes, easiest: avg={filtered[0]['avgGradePct']}%, max={filtered[0]['maxGradePct']}%")
        return jsonify({
            "flatRoute": flat,
            "shortRoute": short,
            "allRoutes": filtered[:6]
        })

    except Exception as e:
        import traceback
        traceback.print_exc()
        return jsonify({"error": str(e)}), 500

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5001)
