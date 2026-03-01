import osmnx as ox
import networkx as nx
from flask import Flask, request, jsonify

app = Flask(__name__)

print("Loading elevation network...")
G = ox.load_graphml(filepath="sf_walk_network_elevation.graphml")

for u, v, k, data in G.edges(keys=True, data=True):
    grade = float(data.get("grade_abs", 0))
    length = float(data.get("length", 0))
    data["impedance"] = length * (1 + 10 * grade ** 2)

print("Network ready. Server starting...")

@app.route("/route", methods=["GET"])
def get_route():
    try:
        start_lat = float(request.args.get("start_lat"))
        start_lng = float(request.args.get("start_lng"))
        end_lat = float(request.args.get("end_lat"))
        end_lng = float(request.args.get("end_lng"))

        origin = ox.distance.nearest_nodes(G, start_lng, start_lat)
        destination = ox.distance.nearest_nodes(G, end_lng, end_lat)

        flat_route = ox.routing.shortest_path(G, origin, destination, weight="impedance")
        short_route = ox.routing.shortest_path(G, origin, destination, weight="length")

        def route_to_coords(route):
            coords = []
            total_gain = 0
            total_length = 0
            for i in range(len(route) - 1):
                u, v = route[i], route[i+1]
                edge_data = G.get_edge_data(u, v)
                edge = edge_data[0] if edge_data else {}
                length = float(edge.get("length", 0))
                grade = float(edge.get("grade", 0))
                rise = length * grade
                if rise > 0:
                    total_gain += rise
                total_length += length
            for node in route:
                node_data = G.nodes[node]
                coords.append({
                    "lat": node_data["y"],
                    "lng": node_data["x"]
                })
            return {
                "coordinates": coords,
                "distanceInMiles": round(total_length / 1609.34, 2),
                "elevationGainFt": round(total_gain * 3.281, 1)
            }

        return jsonify({
            "flatRoute": route_to_coords(flat_route),
            "shortRoute": route_to_coords(short_route)
        })

    except Exception as e:
        return jsonify({"error": str(e)}), 500

if __name__ == "__main__":
    app.run(host="0.0.0.0", port=5001)
