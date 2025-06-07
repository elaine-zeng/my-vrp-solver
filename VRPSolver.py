import pandas as pd
import numpy as np
import requests
import time
from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp
import folium
import warnings
warnings.filterwarnings('ignore')

class VRPSolver:
    def __init__(self):
        self.addresses = [] # string addresses
        self.coordinates = [] # long lat coords
        self.distance_matrix = None
        self.road_paths = {}  # Store road geometries: {(from_idx, to_idx): [(lat, lon), ...]}
        self.num_vehicles = 0
        self.routes = []
        self.UNROUTABLE_DISTANCE_M = 1_000_000_000 # 1,000,000 km in meters.

    def geocode_address(self, address, retries=3, delay=1):
        """
        Geocode an address using Nominatim (OpenStreetMap) API
        """
        base_url = "https://nominatim.openstreetmap.org/search"
        params = {
            'q': address,
            'format': 'json',
            'limit': 1
        }
        for attempt in range(retries):
            try:
                response = requests.get(base_url, params=params, headers={'User-Agent': 'VRP-Solver-App/1.0'})
                response.raise_for_status()
                data = response.json()
                if data:
                    lat = float(data[0]['lat'])
                    lon = float(data[0]['lon'])
                    return lat, lon
                else:
                    print(f"Warning: Could not geocode address: {address}")
                    return None, None
            except Exception as e:
                print(f"Attempt {attempt + 1} failed for {address}: {e}")
                if attempt < retries - 1:
                    time.sleep(delay)
                else:
                    print(f"Failed to geocode after {retries} attempts: {address}")
                    return None, None
        return None, None

    def geocode_addresses(self, addresses):
        if addresses == self.addresses and self.coordinates:
            print("Addresses already geocoded. Skipping re-geocoding.")
            return self.coordinates, []

        print("Geocoding addresses...")
        self.addresses = addresses
        self.coordinates = []
        failed_addresses = []

        geocode_delay_seconds = 1.1 # 1 second per request minimum for Nominatim
        
        for i, address in enumerate(addresses):
            print(f"Geocoding {i+1}/{len(addresses)}: {address}")
            lat, lon = self.geocode_address(address)
            if lat is not None and lon is not None:
                self.coordinates.append((lat, lon))
            else:
                self.coordinates.append((None, None))
                failed_addresses.append(address)
            
            time.sleep(geocode_delay_seconds)

        valid_coords_with_addresses = [(addr, coord) for addr, coord in zip(self.addresses, self.coordinates) if coord[0] is not None and coord[1] is not None]

        self.addresses = [addr for addr, _ in valid_coords_with_addresses]
        self.coordinates = [coord for _, coord in valid_coords_with_addresses]
        
        if failed_addresses:
            print(f"Warning: Failed to geocode the following addresses: {', '.join(failed_addresses)}")
            print(f"Successfully geocoded {len(self.coordinates)} out of {len(addresses)} initial addresses.")
            return self.coordinates, failed_addresses
        else:
            print(f"Successfully geocoded all {len(self.coordinates)} addresses.")
            return self.coordinates, []

    def get_road_distance(self, coord1, coord2, retries=3):
        """
        Get actual road distance between two coordinates using OSRM API
        Returns distance in kilometers and path geometry
        If OSRM fails, returns None, None.
        """
        lon1, lat1 = coord1[1], coord1[0]
        lon2, lat2 = coord2[1], coord2[0]
        url = f"http://router.project-osrm.org/route/v1/driving/{lon1},{lat1};{lon2},{lat2}"
        params = {
            'overview': 'full',
            'geometries': 'geojson'
        }
        
        for attempt in range(retries):
            try:
                response = requests.get(url, params=params, timeout=10)
                response.raise_for_status()
                data = response.json()
                if data['code'] == 'Ok' and data['routes']:
                    distance_m = data['routes'][0]['distance']
                    geometry = data['routes'][0]['geometry']['coordinates']
                    path_coords = [(point[1], point[0]) for point in geometry]
                    return distance_m / 1000, path_coords
                else:
                    print(f"OSRM routing failed between {coord1} and {coord2} (Code: {data.get('code')})")
                    return None, None
            except requests.exceptions.RequestException as e:
                print(f"Attempt {attempt + 1} failed for road distance between {coord1} and {coord2}: {e}")
                if attempt < retries - 1:
                    time.sleep(1)
                else:
                    print(f"Failed to get road distance after {retries} attempts: {coord1} to {coord2}. Setting as unroutable.")
                    return None, None
            except Exception as e:
                print(f"An unexpected error occurred getting road distance: {e}")
                return None, None
        return None, None

    def create_distance_matrix(self):
        if self.distance_matrix is not None and len(self.distance_matrix) == len(self.coordinates):
            print("Distance matrix already created. Skipping re-creation.")
            yield "Distance matrix already created." # For Streamlit
            return self.distance_matrix

        print("Creating distance matrix using ROAD distances (OSRM API).")
        n = len(self.coordinates)
        self.distance_matrix = np.zeros((n, n))
        self.road_paths = {}

        total_pairs = n * (n - 1)
        current_pair = 0
        osrm_delay_seconds = 0.15

        for i in range(n):
            for j in range(n):
                if i == j:
                    self.distance_matrix[i][j] = 0
                else:
                    current_pair += 1
                    yield f"Getting road distance {current_pair}/{total_pairs}: {self.addresses[i]} -> {self.addresses[j]}"

                    dist_km, path_coords = self.get_road_distance(self.coordinates[i], self.coordinates[j])

                    if dist_km is not None:
                        self.distance_matrix[i][j] = int(dist_km * 1000)
                        self.road_paths[(i, j)] = path_coords
                    else:
                        print(f"WARNING: Failed to get road distance for {self.addresses[i]} -> {self.addresses[j]}. Setting to UNROUTABLE_DISTANCE.")
                        self.distance_matrix[i][j] = self.UNROUTABLE_DISTANCE_M
                        self.road_paths[(i, j)] = [self.coordinates[i], self.coordinates[j]]

                    time.sleep(osrm_delay_seconds)

        print(f"Distance matrix created: {n}x{n} using road distances.")
        yield "Distance matrix creation complete."
        return self.distance_matrix

    def solve_vrp(self, num_vehicles, depot_index=0):
        self.num_vehicles = num_vehicles
        
        matrix_gen = self.create_distance_matrix()
        for msg in matrix_gen:
            print(msg)

        print(f"Solving VRP for {num_vehicles} vehicles...")

        manager = pywrapcp.RoutingIndexManager(
            len(self.distance_matrix), num_vehicles, depot_index)

        routing = pywrapcp.RoutingModel(manager)

        def distance_callback(from_index, to_index):
            from_node = manager.IndexToNode(from_index)
            to_node = manager.IndexToNode(to_index)
            cost = int(self.distance_matrix[from_node][to_node])
            return cost

        transit_callback_index = routing.RegisterTransitCallback(distance_callback)
        routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

        dimension_name = 'Distance'
        max_total_distance_per_vehicle = int(self.UNROUTABLE_DISTANCE_M * 1.1)
        routing.AddDimension(
            transit_callback_index,
            0,
            max_total_distance_per_vehicle,
            True,
            dimension_name
        )
        distance_dimension = routing.GetDimensionOrDie(dimension_name)
        distance_dimension.SetGlobalSpanCostCoefficient(1000)

        # --- Constraint to force all vehicles to be utilized ---
        # Add a 'Visits' dimension to ensure each vehicle visits at least one customer.
        def visits_callback(from_index, to_index):
            """Returns 1 if the 'to_node' is a customer (non-depot), 0 otherwise."""
            to_node = manager.IndexToNode(to_index)
            if to_node != depot_index:
                return 1  # This is a visit to a customer
            return 0  # Depot visit or start of route

        visits_callback_index = routing.RegisterTransitCallback(visits_callback)

        routing.AddDimension(
            visits_callback_index,
            0,  # No minimum for individual visit arcs
            len(self.distance_matrix) - 1, # Max capacity a vehicle can visit (all customers)
            True, # True: start value is 0. Important for cumul variables.
            'Visits'
        )
        visits_dimension = routing.GetDimensionOrDie('Visits')

        # For each vehicle, ensure that its total 'visits' (i.e., non-depot stops) is at least 1.
        for vehicle_id in range(num_vehicles):
            total_visits_for_vehicle = visits_dimension.CumulVar(routing.End(vehicle_id))
            # Add the constraint: total_visits_for_vehicle >= 1
            # This forces each vehicle to visit at least one non-depot node.
            routing.solver().Add(total_visits_for_vehicle >= 1)
        # ----------------------------------------------------

        search_parameters = pywrapcp.DefaultRoutingSearchParameters()
        search_parameters.first_solution_strategy = (
            routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
        search_parameters.local_search_metaheuristic = (
            routing_enums_pb2.LocalSearchMetaheuristic.GUIDED_LOCAL_SEARCH)
        search_parameters.time_limit.FromSeconds(60)
        search_parameters.solution_limit = 5000000 # Increased for potentially harder problems with more constraints

        print("Optimizing routes to minimize maximum distance and utilize all vehicles...")
        solution = routing.SolveWithParameters(search_parameters)

        if solution:
            self.routes = self._extract_and_process_routes(manager, routing, solution)
            return self.routes
        else:
            print("No solution found by OR-Tools solver. This might happen if constraints are too restrictive (e.g., trying to use too many vehicles for too few customers).")
            self.routes = [] # Clear routes if no solution
            return None

    def _extract_and_process_routes(self, manager, routing, solution):
        routes_data = []
        for vehicle_id in range(self.num_vehicles):
            index = routing.Start(vehicle_id)
            route_node_indices = []
            route_path_coords = []
            route_distance_m = 0

            # Start node (depot)
            current_node_index = manager.IndexToNode(index)
            route_node_indices.append(current_node_index)
            route_path_coords.append(self.coordinates[current_node_index])

            # Traverse the route
            while not routing.IsEnd(index):
                previous_index = index
                index = solution.Value(routing.NextVar(index))
                next_node_index = manager.IndexToNode(index)

                route_node_indices.append(next_node_index)
                current_segment_distance = routing.GetArcCostForVehicle(previous_index, index, vehicle_id)
                route_distance_m += current_segment_distance

                segment_path = self.road_paths.get((manager.IndexToNode(previous_index), next_node_index),
                                                    [self.coordinates[manager.IndexToNode(previous_index)], self.coordinates[next_node_index]])

                # Append segment path, avoiding duplicate connecting points
                if route_path_coords and segment_path:
                    route_path_coords.extend(segment_path[1:])
                elif segment_path: # Should not happen if route_path_coords is empty, but for safety
                    route_path_coords.extend(segment_path)
            
            routes_data.append({
                'vehicle_id': vehicle_id,
                'route_node_indices': route_node_indices,
                'distance_m': route_distance_m,
                'addresses': [self.addresses[i] for i in route_node_indices],
                'path_coords': route_path_coords
            })
        return routes_data

    def _print_solution_summary(self):
        if not self.routes:
            return "No routes data available."

        output = "\n" + "="*50 + "\n"
        output += "ROUTES OPTIMIZED TO MINIMIZE MAXIMUM DISTANCE (ALL VEHICLES UTILIZED)\n"
        output += "="*50 + "\n"

        route_distances_km = []
        for route_info in self.routes:
            distance_km = route_info['distance_m'] / 1000
            route_distances_km.append(distance_km)

            plan_output = f'Route for vehicle {route_info["vehicle_id"]}:\n'
            plan_output += ' -> '.join(route_info['addresses'])
            plan_output += f'\nDistance: {distance_km:.2f} km\n'
            output += plan_output

        output += "="*50 + "\n"
        
        # Handle case where all routes might be empty (unlikely with new constraint, but good for robustness)
        if not route_distances_km:
            output += "No routes found for summary.\n"
            return output

        max_distance_km = max(route_distances_km)
        min_distance_km = min(route_distances_km)
        avg_distance_km = np.mean(route_distances_km)
        total_distance_km = sum(route_distances_km)

        output += f'MAXIMUM route distance: {max_distance_km:.2f} km\n'
        output += f'MINIMUM route distance: {min_distance_km:.2f} km\n'
        output += f'AVERAGE route distance: {avg_distance_km:.2f} km\n'
        output += f'TOTAL distance across all vehicles: {total_distance_km:.2f} km\n'

        if min_distance_km > 0:
            output += f'Route balance (max/min ratio): {max_distance_km / min_distance_km:.2f}\n'
        else:
            # If min_distance_km is 0, check if all routes are 0
            if all(d == 0 for d in route_distances_km):
                output += "Route balance (max/min ratio): N/A (All routes have zero distance - possibly no customers assigned or all vehicles only go to depot)\n"
            else:
                output += "Route balance (max/min ratio): N/A (Minimum route distance is zero, but other routes are non-zero)\n"
        output += "Lower ratio = better balanced routes\n"
        output += "="*50 + "\n"
        return output

    def create_route_map(self, zoom_start=10):
        if not self.routes or not self.coordinates:
            return None

        # --- Modifications Start Here ---

        # 1. Collect all relevant coordinates for fitting the bounds
        all_lats = []
        all_lons = []

        # Add all unique coordinates from self.coordinates (depot and all possible stops)
        # It's safer to iterate through self.coordinates directly, as some might not be in a route
        for lat, lon in self.coordinates:
            if lat is not None and lon is not None:
                all_lats.append(lat)
                all_lons.append(lon)

        # Add all coordinates from the polyline paths
        for route_info in self.routes:
            for path_coord in route_info['path_coords']:
                all_lats.append(path_coord[0])
                all_lons.append(path_coord[1])

        # Initialize map with a default location/zoom
        # The fit_bounds will override the zoom, but a reasonable starting point is good.
        # If there are no coordinates, default to a generic center.
        if all_lats and all_lons:
            initial_center_lat = np.mean(all_lats)
            initial_center_lon = np.mean(all_lons)
        else:
            initial_center_lat = 0
            initial_center_lon = 0

        m = folium.Map(location=[initial_center_lat, initial_center_lon], zoom_start=zoom_start)


        colors = ['red', 'blue', 'green', 'purple', 'orange', 'darkred', 'lightred', 'beige', 'darkblue', 'darkgreen', 'cadetblue', 'darkpurple', 'white', 'pink', 'lightblue', 'lightgreen', 'gray', 'black', 'lightgray']

        # Add depot marker
        depot_coord = self.coordinates[0]
        folium.Marker(
            depot_coord,
            popup=f'Depot: {self.addresses[0]}',
            tooltip='Depot',
            icon=folium.Icon(color='black', icon='home')
        ).add_to(m)

        for i, route_info in enumerate(self.routes):
            color = colors[i % len(colors)]
            
            # Add markers for each stop (excluding the depot)
            # The route_node_indices includes the start depot and end depot.
            # We want to mark the intermediate customer stops.
            for j, node_idx in enumerate(route_info['route_node_indices']):
                # Ensure node_idx is valid and not the depot (if it's not the first element)
                # The first element is always the depot, so we only need to check if node_idx != 0
                if node_idx != 0:
                    coord = self.coordinates[node_idx]
                    folium.Marker(
                        coord,
                        popup=f"Vehicle {route_info['vehicle_id']} Stop {j}: {self.addresses[node_idx]}",
                        tooltip=f"Stop {j}",
                        icon=folium.Icon(color=color, icon='info-sign')
                    ).add_to(m)

            folium.PolyLine(
                route_info['path_coords'],
                color=color,
                weight=3,
                opacity=0.8,
                popup=f"Vehicle {route_info['vehicle_id']} - {route_info['distance_m']/1000:.2f} km"
            ).add_to(m)

        # 2. Calculate min/max latitude and longitude for the bounds
        if all_lats and all_lons: # Only fit bounds if there are actual coordinates
            min_lat = np.min(all_lats)
            max_lat = np.max(all_lats)
            min_lon = np.min(all_lons)
            max_lon = np.max(all_lons)

            bounds = [[min_lat, min_lon], [max_lat, max_lon]]

            # 3. Apply fit_bounds to the map
            # Add some padding (in pixels) to prevent elements from being right at the edge
            m.fit_bounds(bounds, padding=(20, 20)) # Increased padding for better visibility

        # --- Modifications End Here ---

        return m

    def get_route_summary(self):
        if not self.routes:
            return "No routes available. Run solve_vrp first."
        summary_data = []
        for route_info in self.routes:
            # Number of stops excluding the start/end depot.
            # A route [depot, stop1, stop2, ..., stopN, depot] has N stops.
            # Length of route_node_indices is N+2. So, N = length - 2.
            num_stops = len(route_info['route_node_indices']) - 2
            
            # If the route is only [depot, depot], num_stops should be 0.
            if len(route_info['route_node_indices']) <= 2: 
                num_stops = 0

            summary_data.append({
                'Vehicle': route_info['vehicle_id'],
                'Stops': num_stops,
                'Distance (km)': round(route_info['distance_m'] / 1000, 2),
                'Addresses': ' -> '.join(route_info['addresses'])
            })
        return pd.DataFrame(summary_data)

    def generate_Maps_link(self, route_info):
        """
        Generates a Google Maps URL for a given route.
        """
        base_url = "https://www.google.com/maps/dir/" # Corrected base URL for directions
        
        # Use coordinates for robustness and precision.
        # Format: "lat,lon" for each point.
        coords_for_url = []
        for node_idx in route_info['route_node_indices']:
            lat, lon = self.coordinates[node_idx]
            coords_for_url.append(f"{lat},{lon}")
        
        # Google Maps allows a maximum of 10 waypoints (stops) between origin and destination.
        # Total: Origin + up to 8 waypoints + Destination = 10.
        # Your `route_node_indices` includes start depot, intermediate stops, and end depot.
        # If the number of points exceeds 10, Google Maps will ignore the extras.
        
        if len(coords_for_url) > 10:
            print(f"Warning: Route for vehicle {route_info['vehicle_id']} has too many stops ({len(coords_for_url)}) for a single Google Maps link. The link might only show the first 10 stops.")
            # We don't truncate here, as Google Maps will handle it by ignoring excess.
            # If strict adherence to 10 stops is needed, you'd implement truncation or splitting.

        link = base_url + '/'.join(coords_for_url)
        return link
